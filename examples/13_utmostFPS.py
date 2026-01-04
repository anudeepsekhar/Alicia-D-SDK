# Copyright (c) 2025 Synria Robotics Co., Ltd.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
#
# Author: Synria Robotics Team
# Website: https://synriarobotics.ai

import serial
import time


class fpsFlay:
    # Fps
    sendNum: int = 0
    readNum: int = 0
    errNum: int = 0
    sendFPS: float = 0.0
    readFPS: float = 0.0

    # Time
    start_time: float = 0.0
    current_time: float = 0.0
    real_time: float = 0.0
    end_time: float = 0.0

    def __init__(self):
        pass

def main():
    TEST_TIMEOUT = 300.0    # s
    Fps = fpsFlay()
    try:
        ser = serial.Serial(
            port="/dev/cu.wchusbserial5B140413941",  # macOS specific port
            baudrate=1000000,
            timeout=0.015,  # Final 1khz update rate
        )
        sendBuff = bytearray([0xAA, 0x06, 0x00, 0x01, 0xFE, 0x9A, 0xFF])

        sendFlay = True
        Fps.start_time = time.time()
        Fps.real_time = Fps.start_time
        while sendFlay:
            ser.write(sendBuff)
            Fps.sendNum += 1
            readBuff = ser.read(21)
            if(len(readBuff) != 21 or readBuff[0] != 0xAA or readBuff[-1] != 0xFF or readBuff[1] != 0x06):
                Fps.errNum += 1
                continue
            Fps.readNum += 1
            Fps.current_time = time.time()
            if Fps.current_time - Fps.real_time >= 1.0:
                if Fps.current_time - Fps.start_time >= TEST_TIMEOUT: 
                    sendFlay = False
                Fps.real_time = Fps.current_time;
                Fps.sendFPS = Fps.sendNum / (Fps.current_time - Fps.start_time)
                Fps.readFPS = Fps.readNum / (Fps.current_time - Fps.start_time)
                print(f"[{Fps.current_time - Fps.start_time:.2f}s]发送帧率: {Fps.sendFPS:.2f}Hz, \
                      接收帧率: {Fps.readFPS:.2f}Hz, \
                        错误数量: {Fps.errNum}")

    except KeyboardInterrupt:
        print(f"Ctrl+C退出")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        Fps.end_time = time.time()
        Fps.sendFPS = Fps.sendNum / (Fps.end_time - Fps.start_time)
        Fps.readFPS = Fps.readNum / (Fps.end_time - Fps.start_time)
        print(f"All Time: {Fps.end_time - Fps.start_time:.2f}s")
        print(f"sendFPS: {Fps.sendFPS:.2f}Hz")
        print(f"readFPS: {Fps.readFPS:.2f}Hz")
        print(f"Sync rate: {(Fps.readNum/Fps.sendNum)*100:.2f}%")
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("程序终止")
