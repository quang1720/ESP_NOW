

import serial
import time
import threading

ser = None
def init_bytes():
    data_ref = []
    for i in range(0,1):
        for j in range(0, 250):
            if i == 1:
                data_ref.append(249 - j)
            else:
                data_ref.append(j)
    # print(data_ref)
    return bytes(data_ref)
data = init_bytes()

def read_serial():
    try:
        while True:
            bytes = ser.read_all()
            if (len(bytes) > 0):
                print(f"{bytes}")
                pass
    except Exception as ex:
        print(f"Read error: {ex}")

def write_serial():
    try:
        while True:
            if data is None: continue
            ser.write(data)
            time.sleep(1/50)
    except Exception as ex:
         print(f"Write error: {ex}")

if __name__ == "__main__":
    try:
        ser = serial.Serial(
            port='COM4',
            baudrate=230400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        threading.Thread(target=read_serial, args=()).start()
        threading.Thread(target=write_serial, args=()).start()
    except KeyboardInterrupt:
        ser.close()
        print("\nSerial port closed. Exiting.")