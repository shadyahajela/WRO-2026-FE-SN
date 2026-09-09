import time
from gpiozero import Button

# Signal wire on GPIO16 (physical pin 36), ground wire on physical pin 34
BUTTON_PIN = 16

button = Button(BUTTON_PIN, bounce_time=0.05)

print(f"Watching button on GPIO{BUTTON_PIN}. Press Ctrl+C to exit.")

try:
    # while True:
    button.wait_for_press()
    print("Button pressed")
    button.wait_for_release()
    print("Button released")

except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    button.close()
    print("Stopped")
