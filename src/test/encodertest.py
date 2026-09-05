import time
import serial
from gpiozero import RotaryEncoder

ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
time.sleep(.5)

# Define GPIO pins for Channel A and Channel B
# These use Broadcom (BCM) pin numbering
PIN_A = 17
PIN_B = 27

# Pulses per one revolution of the output (gearbox) shaft - check your specific
# JGA25-371 variant's datasheet (encoder disc PPR x gear ratio). Placeholder value.
PULSES_PER_REV = 1
#990

# Initialize the rotary encoder
# max_steps=0 removes the upper boundary limit for continuous counting
encoder = RotaryEncoder(PIN_A, PIN_B, max_steps=0)

center = 100       # steering value: straight ahead
speed_value = 255  # top speed
on = 1             # motor direction: 1 - forward

run_message = f"{center} {speed_value} {on} 0 RUN\n"
stop_message = f"{center} 0 {on} 0 RUN\n"

print("Motor running at top speed")
print("Reading JGA25-370 Encoder... Press Ctrl+C to exit.")

last_count = 0
last_time = time.monotonic()

try:
    while True:
        ser.write(run_message.encode())
        ser.flush()       
        # encoder.steps tracks the position (increases or decreases)
        # print(f"Pulses: {encoder.steps}")
        # time.sleep(0.1)

        count = encoder.steps

        now = time.monotonic()
        dt = now - last_time
        delta = count - last_count
        rpm = (delta / PULSES_PER_REV) / (dt / 60)
        last_count = count
        last_time = now

        print(f"Encoder count: {count}  RPM: {rpm:.1f}")
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
    encoder.close()
finally:
    ser.write(stop_message.encode())
    ser.flush()
    ser.close()
    print("\nStopped")
