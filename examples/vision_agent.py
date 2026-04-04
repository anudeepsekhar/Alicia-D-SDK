"""
Vision Agent — Claude-powered robot arm controller with camera feedback.

Uses the Anthropic tool-use API to run a perception-action loop:
  1. User says what to fetch
  2. Agent captures camera image, reasons about the scene
  3. Agent calls robot tools (move, gripper, etc.) in small increments
  4. Repeats until the task is done

Usage:
    export ANTHROPIC_API_KEY=sk-ant-...
    python examples/vision_agent.py
"""

import os
import sys
import math
import time
import base64
import json

import cv2
import anthropic
import alicia_d_sdk

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

MODEL = "claude-haiku-4-5-20251001"
MAX_AGENT_STEPS = 30
SETTLE_TIME = 0.5  # seconds to wait after a move command
CAMERA_INDEX = 0

SYSTEM_PROMPT = """\
You control a 6-DOF robot arm with a workspace camera. Be FAST. Take small, quick steps.

## Joints
- j1: Base rotation (positive=right)  |  j2: Shoulder (positive=UP)  |  j3: Elbow (negative=UP)
- j4, j5, j6: Wrist joints

## Safety limits
- j2: -10 to 50  |  j3: -50 to 10  |  All: -90 to 90

## CRITICAL RULES
- Use move_relative for EVERY move. Steps of 5-15 degrees.
- NEVER call capture_image twice in a row. Always move between captures.
- NEVER explain your reasoning. Output ONLY the tool call, no text at all.
- Complete the task in under 15 steps.

## Strategy
1. capture_image → find object direction.
2. move_relative dj1 toward object (10-20 deg). Move + capture.
3. move_relative dj2 down, dj3 to extend. Move + capture.
4. Small adjustments, then close_gripper.
5. move_relative dj2 +20 to lift. go_home. done.
"""

# ---------------------------------------------------------------------------
# Tool definitions for Claude
# ---------------------------------------------------------------------------

TOOLS = [
    {
        "name": "capture_image",
        "description": (
            "Capture an image from the workspace camera. Returns the image so you "
            "can see the current scene. Call this frequently to track progress."
        ),
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "move_joints",
        "description": (
            "Move the arm to absolute joint angles (degrees). All 6 joints must be "
            "specified. Respects safety limits. The arm will move and settle."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "j1": {"type": "number", "description": "Base rotation (deg)"},
                "j2": {"type": "number", "description": "Shoulder (deg, positive=up)"},
                "j3": {"type": "number", "description": "Elbow (deg, negative=up)"},
                "j4": {"type": "number", "description": "Wrist 1 (deg)"},
                "j5": {"type": "number", "description": "Wrist 2 (deg)"},
                "j6": {"type": "number", "description": "Wrist 3 (deg)"},
            },
            "required": ["j1", "j2", "j3", "j4", "j5", "j6"],
        },
    },
    {
        "name": "move_relative",
        "description": (
            "Move joints by relative amounts (degrees) from the current position. "
            "Safer for incremental adjustments. Unspecified joints default to 0 delta."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "dj1": {"type": "number", "description": "Base delta (deg)", "default": 0},
                "dj2": {"type": "number", "description": "Shoulder delta (deg)", "default": 0},
                "dj3": {"type": "number", "description": "Elbow delta (deg)", "default": 0},
                "dj4": {"type": "number", "description": "Wrist 1 delta (deg)", "default": 0},
                "dj5": {"type": "number", "description": "Wrist 2 delta (deg)", "default": 0},
                "dj6": {"type": "number", "description": "Wrist 3 delta (deg)", "default": 0},
            },
            "required": [],
        },
    },
    {
        "name": "open_gripper",
        "description": "Open the gripper fully.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "close_gripper",
        "description": "Close the gripper to grasp an object.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "get_joint_state",
        "description": "Get current joint angles in degrees and gripper value.",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "go_home",
        "description": "Move the arm to the home position (all joints at 0 degrees).",
        "input_schema": {"type": "object", "properties": {}, "required": []},
    },
    {
        "name": "set_speed",
        "description": "Set servo speed in degrees per second (1-360).",
        "input_schema": {
            "type": "object",
            "properties": {
                "deg_s": {"type": "number", "description": "Speed in deg/s"},
            },
            "required": ["deg_s"],
        },
    },
    {
        "name": "done",
        "description": (
            "Signal that the task is complete (or cannot be completed). "
            "Provide a short message for the user."
        ),
        "input_schema": {
            "type": "object",
            "properties": {
                "message": {"type": "string", "description": "Summary for the user"},
            },
            "required": ["message"],
        },
    },
]

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

robot = None
camera = None
client = None


def init():
    """Initialize robot, camera, and Anthropic client."""
    global robot, camera, client

    api_key = os.environ.get("ANTHROPIC_API_KEY")
    if not api_key:
        print("ERROR: Set ANTHROPIC_API_KEY environment variable.")
        sys.exit(1)

    client = anthropic.Anthropic(api_key=api_key)

    print("Connecting to robot...")
    robot = alicia_d_sdk.create_robot()
    print("Robot connected.")

    print(f"Opening camera (index {CAMERA_INDEX})...")
    camera = cv2.VideoCapture(CAMERA_INDEX)
    if not camera.isOpened():
        print(f"ERROR: Cannot open camera at index {CAMERA_INDEX}")
        sys.exit(1)
    # Warm up the camera with a few dummy reads
    for _ in range(5):
        camera.read()
        time.sleep(0.1)
    print("Camera ready.")


def shutdown():
    """Clean up resources."""
    global robot, camera
    if robot:
        try:
            robot.set_robot_state(
                target_joints=[0, 0, 0, 0, 0, 0],
                joint_format="deg",
                wait_for_completion=False,
            )
            time.sleep(2)
            robot.disconnect()
        except Exception:
            pass
    if camera:
        camera.release()


# ---------------------------------------------------------------------------
# Tool implementations
# ---------------------------------------------------------------------------

JOINT_LIMITS = {
    "j2_min": -10, "j2_max": 50,
    "j3_min": -50, "j3_max": 10,
}


def clamp_joints(joints):
    """Apply safety limits to joint angles."""
    j = list(joints)
    j[1] = max(JOINT_LIMITS["j2_min"], min(JOINT_LIMITS["j2_max"], j[1]))
    j[2] = max(JOINT_LIMITS["j3_min"], min(JOINT_LIMITS["j3_max"], j[2]))
    for i in range(6):
        j[i] = max(-90, min(90, j[i]))
    return j


def tool_capture_image():
    """Capture a frame, resize for speed, and return as base64 JPEG."""
    ret, frame = camera.read()
    if not ret:
        return {"type": "text", "content": "ERROR: Failed to capture image"}
    h, w = frame.shape[:2]
    if w > 480:
        scale = 480 / w
        frame = cv2.resize(frame, (480, int(h * scale)))
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
    b64 = base64.standard_b64encode(buf).decode("utf-8")
    kb = len(b64) * 3 / 4 / 1024
    print(f"         (image: {frame.shape[1]}x{frame.shape[0]}, {kb:.0f} KB)")
    return {
        "type": "image",
        "b64": b64,
    }


def tool_move_joints(j1, j2, j3, j4, j5, j6):
    target = clamp_joints([j1, j2, j3, j4, j5, j6])
    robot.set_robot_state(
        target_joints=target,
        joint_format="deg",
        speed_deg_s=60,
        wait_for_completion=False,
    )
    time.sleep(SETTLE_TIME)
    cur = robot.get_robot_state("joint")
    if cur:
        degs = [round(math.degrees(a), 1) for a in cur]
        return f"Moved. Current joints (deg): {degs}"
    return "Moved (could not read back state)."


def tool_move_relative(dj1=0, dj2=0, dj3=0, dj4=0, dj5=0, dj6=0):
    cur = robot.get_robot_state("joint")
    if not cur:
        return "ERROR: Cannot read current joint state."
    cur_deg = [math.degrees(a) for a in cur]
    target = [
        cur_deg[0] + dj1,
        cur_deg[1] + dj2,
        cur_deg[2] + dj3,
        cur_deg[3] + dj4,
        cur_deg[4] + dj5,
        cur_deg[5] + dj6,
    ]
    return tool_move_joints(*target)


def tool_open_gripper():
    robot.set_robot_state(gripper_value=1000, wait_for_completion=False)
    time.sleep(0.5)
    return "Gripper opened."


def tool_close_gripper():
    robot.set_robot_state(gripper_value=0, wait_for_completion=False)
    time.sleep(0.5)
    return "Gripper closed."


def tool_get_joint_state():
    cur = robot.get_robot_state("joint")
    gripper = robot.get_robot_state("gripper")
    if cur:
        degs = [round(math.degrees(a), 1) for a in cur]
        return f"Joints (deg): {degs}, Gripper: {gripper}"
    return "ERROR: Cannot read state."


def tool_go_home():
    robot.set_robot_state(
        target_joints=[0, 0, 0, 0, 0, 0],
        joint_format="deg",
        wait_for_completion=False,
    )
    time.sleep(1.0)
    return "Returned to home position."


def tool_set_speed(deg_s):
    deg_s = max(1, min(360, deg_s))
    robot.set_servo_speed(deg_s)
    return f"Speed set to {deg_s} deg/s."


def _dispatch_tool(name, input_args):
    """Route tool call to implementation."""
    if name == "capture_image":
        return tool_capture_image()
    elif name == "move_joints":
        return tool_move_joints(
            input_args["j1"], input_args["j2"], input_args["j3"],
            input_args["j4"], input_args["j5"], input_args["j6"],
        )
    elif name == "move_relative":
        return tool_move_relative(
            input_args.get("dj1", 0), input_args.get("dj2", 0),
            input_args.get("dj3", 0), input_args.get("dj4", 0),
            input_args.get("dj5", 0), input_args.get("dj6", 0),
        )
    elif name == "open_gripper":
        return tool_open_gripper()
    elif name == "close_gripper":
        return tool_close_gripper()
    elif name == "get_joint_state":
        return tool_get_joint_state()
    elif name == "go_home":
        return tool_go_home()
    elif name == "set_speed":
        return tool_set_speed(input_args["deg_s"])
    elif name == "done":
        return input_args.get("message", "Done.")
    else:
        return f"Unknown tool: {name}"


def execute_tool(name, input_args):
    """Dispatch a tool call, print timing."""
    short_args = json.dumps(input_args) if input_args else ""
    if len(short_args) > 80:
        short_args = short_args[:77] + "..."
    print(f"  [TOOL] {name}({short_args})", flush=True)
    t0 = time.time()
    result = _dispatch_tool(name, input_args)
    dt = time.time() - t0
    print(f"         -> {dt:.1f}s", flush=True)
    return result


# ---------------------------------------------------------------------------
# Agent loop
# ---------------------------------------------------------------------------

MAX_HISTORY = 3


def _trim_messages(messages):
    """Keep first user message + last MAX_HISTORY turns, strip old images."""
    if len(messages) <= 1 + MAX_HISTORY * 2:
        trimmed = [m for m in messages]
    else:
        trimmed = [messages[0]] + messages[-(MAX_HISTORY * 2):]

    # Strip all images except in the very last user message
    for i in range(len(trimmed) - 1):
        msg = trimmed[i]
        if msg.get("role") != "user":
            continue
        content = msg.get("content")
        if not isinstance(content, list):
            continue
        for j, block in enumerate(content):
            if isinstance(block, dict) and block.get("type") == "tool_result":
                inner = block.get("content")
                if isinstance(inner, list) and any(
                    isinstance(item, dict) and item.get("type") == "image"
                    for item in inner
                ):
                    trimmed[i]["content"][j] = {
                        "type": "tool_result",
                        "tool_use_id": block["tool_use_id"],
                        "content": "[image seen earlier]",
                    }
    return trimmed


def run_agent(user_request: str):
    """Run the perception-action loop for a single user request."""
    messages = [
        {
            "role": "user",
            "content": (
                f"The user wants you to: {user_request}\n\n"
                "Start by capturing an image to see the workspace, then work toward the goal."
            ),
        }
    ]

    task_done = False
    steps = 0

    task_start = time.time()

    while not task_done and steps < MAX_AGENT_STEPS:
        steps += 1
        elapsed = time.time() - task_start
        print(f"\n--- Step {steps} [{elapsed:.0f}s elapsed] ---", flush=True)

        trimmed = _trim_messages(messages)

        t_api = time.time()
        # Stream the response so we see text in real-time and can act on tools faster
        print("  ", end="", flush=True)
        with client.messages.stream(
            model=MODEL,
            max_tokens=512,
            system=SYSTEM_PROMPT,
            tools=TOOLS,
            messages=trimmed,
        ) as stream:
            for text in stream.text_stream:
                print(text, end="", flush=True)
            response = stream.get_final_message()
        dt_api = time.time() - t_api
        # Print newline after streamed text if any text was output
        has_text = any(b.type == "text" for b in response.content)
        if has_text:
            print(flush=True)
        print(f"  [{dt_api:.1f}s, {len(trimmed)} msgs]", flush=True)

        assistant_content = response.content
        messages.append({"role": "assistant", "content": assistant_content})

        tool_blocks = [b for b in assistant_content if b.type == "tool_use"]
        if not tool_blocks:
            print("  (No tool calls — agent finished)")
            break

        tool_results = []
        for tool_block in tool_blocks:
            result = execute_tool(tool_block.name, tool_block.input)

            if tool_block.name == "done":
                print(f"\n  [DONE] {result}")
                task_done = True
                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_block.id,
                    "content": result,
                })
                break

            if isinstance(result, dict) and result.get("type") == "image":
                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_block.id,
                    "content": [
                        {
                            "type": "image",
                            "source": {
                                "type": "base64",
                                "media_type": "image/jpeg",
                                "data": result["b64"],
                            },
                        }
                    ],
                })
            else:
                text_content = result if isinstance(result, str) else json.dumps(result)
                tool_results.append({
                    "type": "tool_result",
                    "tool_use_id": tool_block.id,
                    "content": text_content,
                })

        messages.append({"role": "user", "content": tool_results})

    if not task_done:
        print(f"\n  Agent hit step limit ({MAX_AGENT_STEPS}).")

    return steps


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    init()
    print("\n=== Robot Vision Agent ===")
    print("Tell the agent what to fetch, or type 'quit' to exit.\n")

    try:
        while True:
            user_input = input("What should I fetch? > ").strip()
            if not user_input:
                continue
            if user_input.lower() in ("quit", "exit", "q"):
                break
            print(f"\nStarting task: {user_input}")
            run_agent(user_input)
            print()
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        print("Shutting down...")
        shutdown()
        print("Done.")


if __name__ == "__main__":
    main()
