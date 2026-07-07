# proportional_test.py

CENTER = 90          # Straight steering
MIN_SERVO = 55
MAX_SERVO = 125

kp = 0.11            # Change this while testing

print("Proportional Steering Test")
print("---------------------------")
print("Enter steering error in pixels.")
print("Negative = wall on left")
print("Positive = wall on right")
print("Type 'q' to quit.\n")

while True:

    user = input("Error: ")

    if user.lower() == "q":
        break

    try:
        error = float(user)

        steering = CENTER + kp * error

        # Clamp servo
        steering = max(MIN_SERVO, min(MAX_SERVO, steering))

        # Round to nearest 5°
        steering = round(steering / 5) * 5

        print(f"Servo = {steering:.0f}°\n")

    except ValueError:
        print("Enter a number.\n")