import serial
import random

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

print("Waiting for requests...")

while True:
    if ser.in_waiting:
        message = ser.readline().decode().strip()

        print("Received:", message)

        if message == "START":
            ser.write(f"115 155 1 12 OPEN\n".encode())
            ser.flush()
        
ser.close()
        

