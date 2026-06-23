import serial
import random

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

print("Waiting for requests...")

while True:
    if ser.in_waiting:
        message = ser.readline().decode().strip()

        print("Received:", message)

        if message == "START":
            value = random.randint(1, 100)

            ser.write(f"{value}\n".encode())

            print("Sent:", value) 
            
