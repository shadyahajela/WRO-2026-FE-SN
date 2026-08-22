import sensor.bno055 as bno
import time

#IMU
bno.initialize() 
initial = bno.get_initial_heading()

try:
    while True:
        cal_status = bno.sensor.calibration_status
        heading = bno.get_relative_heading(initial)
        if heading is not None:
            print(f"\r {heading:7.2f}" , end="")
        else:
            print("\rCould not read heading.", end="")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nTest interrupted by user.")
finally:
    bno.cleanup()