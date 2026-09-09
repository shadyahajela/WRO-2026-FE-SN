import serial
import time

from gpiozero import Button

ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)

commands = [
    f"$ 108\n",
    f"$ 90\n",
    f"$ 108\n",
    f"$ 90\n",
    f"$ 100\n"
]

print("sending...")

time.sleep(5)

ser.write("# OPIN\n".encode())
ser.flush()

time.sleep(1)

#start button - signal wire on GPIO16 (physical pin 36), ground wire on physical pin 34
BUTTON_PIN = 16
start_button = Button(BUTTON_PIN, bounce_time=0.05)

print(f"Waiting for start button release on GPIO{BUTTON_PIN}...")
start_button.wait_for_press()
print("Button pressed")
start_button.wait_for_release()
print("Button released - starting run")

ser.write("# VRMM\n".encode())
ser.flush()

while True:

    for each in commands:
        print(each)
        ser.write(each.encode())
        ser.flush()
        time.sleep(0.5)

        
ser.close()
        

