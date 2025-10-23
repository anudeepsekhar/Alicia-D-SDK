"""
Demo: 读取机械臂固件版本号

Updated for Alicia-D SDK v6.0.0 with RoboCore bridge
"""

import time
import threading
import alicia_d_sdk
from alicia_d_sdk.utils.logger import logger

def sender_thread(robot, stop_event, send_interval=0.5):
    """Thread for sending version read commands periodically."""
    command = [0xAA, 0x0A, 0x01, 0x00, 0x00, 0xFF]
    
    while not stop_event.is_set():
        try:
            robot.servo_driver.serial_comm.send_data(command)
        except Exception as e:
            print(f"Error sending command: {e}")
        
        # Wait for the specified interval or until stop event
        if stop_event.wait(send_interval):
            break
    

def receiver_thread(robot, firmware_version, stop_event, check_interval=0.1):
    """Thread for receiving and checking firmware version."""
    
    while not stop_event.is_set():
        try:
            version = robot.get_firmware_version()
            if version and version != "未知版本":
                firmware_version[0] = version
                stop_event.set()  # Signal both threads to stop
                break
        except Exception as e:
            logger.error(f"Error receiving firmware version: {e}")
        
        # Wait for the specified interval or until stop event
        if stop_event.wait(check_interval):
            break
    

def main(args):
    """Read and print robot firmware version using two threads.
    
    :param args: Command line arguments containing port, baudrate, version, and gripper_type
    """
    # Initialize robot instance
    robot = alicia_d_sdk.create_robot(
        port=args.port,
        baudrate=args.baudrate,
        robot_version=args.version,
        gripper_type=args.gripper_type
    )
    
    # Shared variables
    firmware_version = [None]  
    stop_event = threading.Event()
    
    try:
        # Connect to robot
        if robot.connect():            
            # Start sender and receiver threads
            sender = threading.Thread(target=sender_thread, args=(robot, stop_event))
            receiver = threading.Thread(target=receiver_thread, args=(robot, firmware_version, stop_event))
            
            sender.start()
            receiver.start()
            
            # Wait for either thread to complete or timeout
            timeout = 10.0  # 10 seconds timeout
            start_time = time.time()
            
            while not stop_event.is_set() and (time.time() - start_time) < timeout:
                time.sleep(0.1)
            
            stop_event.set()
            
            sender.join(timeout=1.0)
            receiver.join(timeout=1.0)
            
            # Check result
            if not firmware_version[0]:
                logger.error("Failed to read firmware version (timeout or error)")

        else:
            logger.error("Connection failed, please check serial port settings")
    
    except KeyboardInterrupt:
        logger.info("\nReading interrupted")
        stop_event.set()
    
    except Exception as e:
        print(f"Error: {e}")
        stop_event.set()
    
    finally:
        stop_event.set()
        robot.disconnect()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description="Read robot firmware version")
    
    # Robot configuration
    parser.add_argument('--port', type=str, default="/dev/ttyCH343USB0", help="串口端口 (例如: /dev/ttyUSB0 或 COM3)")
    parser.add_argument('--baudrate', type=int, default=1000000,  help="波特率 (默认: 1000000)")
    parser.add_argument('--version', type=str, default="v5_6",  help="机械臂版本 (默认: v5_6)")
    parser.add_argument('--gripper_type', type=str, default="50mm",  help="夹爪型号 (默认: 50mm)")
    parser.add_argument('--speed', type=float, default=1,  help="运动速度因子 (0.0 ~ 1.0, 默认: 0.5)")

    args = parser.parse_args()
    
    
    main(args)
