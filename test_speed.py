"""Test speed + acceleration commands then move."""
import serial
import time
import math
import struct

PORT = "/dev/cu.wchusbserial5B140413001"
p = serial.Serial(PORT, 1000000, timeout=0.2)
p.reset_input_buffer()
time.sleep(0.3)


def build_v5_frame(cmd, data):
    frame = [0xAA, cmd] + [len(data)] + data + [0, 0xFF]
    crc = sum(frame[3:-2]) % 2
    frame[-2] = crc
    return bytes(frame)


def r2h(angle_deg):
    angle_rad = math.radians(angle_deg)
    return max(0, min(4095, int((angle_rad + math.pi) / (2 * math.pi) * 4096)))


def build_joint_cmd(joints_deg):
    jmap = [(0, 1), (0, 1), (1, 1), (1, -1), (2, 1), (2, -1), (3, 1), (4, 1), (5, 1)]
    data = []
    for ji, d in jmap:
        hw = r2h(joints_deg[ji] * d)
        data.append(hw & 0xFF)
        data.append((hw >> 8) & 0xFF)
    return build_v5_frame(0x04, data)


def speed_to_hw(deg_s):
    rad_s = math.radians(deg_s)
    raw = int((rad_s / (2 * math.pi)) * 3400)
    return max(1, min(3400, raw))


def build_speed_cmd(deg_s):
    hw = speed_to_hw(deg_s)
    data = [0x2E]
    for _ in range(10):
        data.append(hw & 0xFF)
        data.append((hw >> 8) & 0xFF)
    return build_v5_frame(0x05, data)


def build_accel_cmd():
    data = [0x29] + [254] * 18
    return build_v5_frame(0x05, data)


def read_pos(port):
    port.reset_input_buffer()
    time.sleep(0.08)
    raw = port.read(300)
    for i in range(len(raw) - 22):
        if raw[i] == 0xAA and raw[i + 1] == 0x14 and raw[i + 2] == 0x12 and raw[i + 22] == 0xFF:
            vals = []
            for j in range(9):
                v = raw[i + 3 + j * 2] | (raw[i + 4 + j * 2] << 8)
                vals.append(round(math.degrees((v / 4096 * 2 * math.pi) - math.pi), 1))
            return vals[:6]
    return None


# Set high acceleration
print("Setting acceleration...")
accel_cmd = build_accel_cmd()
print(f"  Frame: {' '.join(f'{b:02X}' for b in accel_cmd)}")
p.write(accel_cmd)
time.sleep(0.1)

# Set speed to 200 deg/s
SPEED = 200
print(f"Setting speed to {SPEED} deg/s...")
speed_cmd = build_speed_cmd(SPEED)
print(f"  Frame: {' '.join(f'{b:02X}' for b in speed_cmd)}")
p.write(speed_cmd)
time.sleep(0.1)

pos0 = read_pos(p)
print(f"Start: {pos0}")

# Move to 40 degrees on joint 1
print(f"\nMoving joint 1 to 40 deg at {SPEED} deg/s...")
cmd = build_joint_cmd([40, 0, 0, 0, 0, 0])
p.write(cmd)

for t in range(12):
    time.sleep(0.5)
    pos = read_pos(p)
    if pos:
        print(f"  t={t*0.5+0.5:.1f}s: j1={pos[0]:6.1f}")

# Move home
print("\nHoming...")
cmd = build_joint_cmd([0, 0, 0, 0, 0, 0])
p.write(cmd)

for t in range(12):
    time.sleep(0.5)
    pos = read_pos(p)
    if pos:
        print(f"  t={t*0.5+0.5:.1f}s: j1={pos[0]:6.1f}")

p.close()
print("Done.")
