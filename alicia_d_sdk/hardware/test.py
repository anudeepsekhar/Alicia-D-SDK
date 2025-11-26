

#################################################
# For servo driver test
#################################################

from servo_driver import ServoDriver
servo_driver = ServoDriver()
servo_driver.connect()
# servo_driver.get_firmware_version()
servo_driver.set_torque(False)

#################################################
# For serial communication test
#################################################

# from serial_comm import SerialComm

# serial_comm = SerialComm()

# serial_comm.connect()

# for i in range(10):
#     data = serial_comm.read_frame()
#     if data:
#         print(data)
#     else:
#         print("No data")