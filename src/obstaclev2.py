#IMPORTS
import time 
import math
import serial

import numpy as np #where you store hue/color values

from frames import Frame #define your color ranges and other frame-related functions

#get camera working
from picamera2 import Picamera2
import cv2

#RED COLOR VALUES (from obstacle_FAKE.py)
# Red wraps around hue 0/180, so two ranges are combined
lowRed1  = np.array([0, 128, 50])
highRed1 = np.array([5, 255, 255])
lowRed2  = np.array([170, 128, 50])
highRed2 = np.array([180, 255, 255])

#GREEN COLOR VALUES (from obstacle_challenge.py)
lowGreen  = np.array([40, 70, 50])
highGreen = np.array([85, 255, 255])

#BLACK WALL COLOR VALUES (from obstacle_challenge.py)
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 80])

#MICROBIT VALUES
center = 100 # base steering value, PID correction is added/subtracted from this
steering_value = 100 # Calculated steering value to add or subtract from center value
prev_steer = steering_value
margin = 55
speed_value = 150 # Speed, 160/135 is lowest, 255 is highest
on = 1

lines = 0

#PID VALUES
kp = 0.09
kd = 0.2
ky = 0.9  # tune how aggressively distance affects steering: >1 = harsher falloff (goes light faster with distance), <1 = gentler falloff
previous_error = 0
previous_error_green = 0

#WALL FOLLOWING VALUES (used only when no red/green obstacle is visible)
wfx1, wfy1, wfx2, wfy2 = 0, 220, 640, 260   # small band - mirrors obstacle_challenge.py's no-obstacle default
wf_gap_left_x = 220
wf_gap_right_x = 420
kp_wall = 0.4
kd_wall = 0.22
dead_zone_px = 20
previous_error_wall = 0

indicator = "abcd"

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = ""

#getting serial connection working
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
time.sleep(2)

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

time.sleep(3)  # Allow camera to warm up 

picam2.set_controls({"Contrast": 1})  # Set frame duration to 30 FPS

#main loop to show camera feed
while True:

    #camera feed
    image = picam2.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    # flip upside down
    image = cv2.rotate(image, cv2.ROTATE_180)

    # pristine snapshot taken before anything is drawn this frame - every Frame's color/contour
    # detection reads from this instead of `image`, so no debug overlay (drawn onto `image` as
    # the loop runs) can ever cut through a block/wall mask that hasn't been detected yet
    image_source = image.copy()

    #detect red across the entire screen and draw a bounding box around the largest blob
    red_frame = Frame(0, 0, 640, 480, image, [lowRed1, lowRed2], [highRed1, highRed2], frameColor=(0,0,255), source=image_source)
    red_contours = red_frame.getColorContours([0,1], min_area=200)

    if red_contours:
        largest_red = max(red_contours, key=cv2.contourArea)
        rx, ry, rw, rh = cv2.boundingRect(largest_red)
        cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), (0,0,255), 2)

        bottom_right_dot = (rx + rw, ry + rh)
        bottom_left_dot = (0, image.shape[0])

        cv2.circle(image, bottom_right_dot, 6, (0,0,255), -1)
        cv2.circle(image, bottom_left_dot, 6, (0,0,255), -1)

        #PID control using the line's two x values as the error
        error = bottom_right_dot[0] - bottom_left_dot[0]

        #PD terms from the x error
        p_term = kp * error
        derivative = error - previous_error
        d_term = kd * derivative
        previous_error = error

        #the closer the block (larger y, nearer the bottom of the screen), the harsher the correction;
        #the farther away (smaller y), the lighter it is
        y_error = bottom_right_dot[1] - bottom_left_dot[1]
        distance = abs(y_error)
        y_factor = max(0, 1 - (ky * distance / image.shape[0]))   # 1.0 = touching the bottom (closest), 0.0 = at/past the falloff point set by ky

        #scale PD output by distance falloff to get final steering value
        control_signal = (p_term + d_term) * y_factor
        steering_value_red = center + control_signal

        #clamp steering to center +/- margin
        steering_value_red = max(center - margin, min(center + margin, steering_value_red))

        #round everything to the nearest whole number
        error = round(error)
        p_term = round(p_term)
        d_term = round(d_term)
        y_error = round(y_error)
        steering_value_red = round(steering_value_red)

        print(f"Right X: {bottom_right_dot[0]}, Left X: {bottom_left_dot[0]}, Y Error: {y_error}, Y Factor: {round(y_factor, 2)}")
        print(f"P: {p_term}, D: {d_term}, Error: {error}, Steering Value: {steering_value_red}")

    #detect green across the entire screen and draw a bounding box around the largest blob
    green_frame = Frame(0, 0, 640, 480, image, [lowGreen], [highGreen], frameColor=(0,255,0), source=image_source)
    green_contours = green_frame.getColorContours(0, min_area=200)

    if green_contours:
        largest_green = max(green_contours, key=cv2.contourArea)
        gx, gy, gw, gh = cv2.boundingRect(largest_green)
        cv2.rectangle(image, (gx, gy), (gx + gw, gy + gh), (0,255,0), 2)

        bottom_left_dot_green = (gx, gy + gh)
        bottom_right_dot_green = (image.shape[1], image.shape[0])

        cv2.circle(image, bottom_left_dot_green, 6, (0,255,0), -1)
        cv2.circle(image, bottom_right_dot_green, 6, (0,255,0), -1)

        #PID control using the line's two x values as the error
        error_green = bottom_right_dot_green[0] - bottom_left_dot_green[0]

        #PD terms from the x error
        p_term_green = kp * error_green
        derivative_green = error_green - previous_error_green
        d_term_green = kd * derivative_green
        previous_error_green = error_green

        #the closer the block (larger y, nearer the bottom of the screen), the harsher the correction;
        #the farther away (smaller y), the lighter it is
        y_error_green = bottom_right_dot_green[1] - bottom_left_dot_green[1]
        distance_green = abs(y_error_green)
        y_factor_green = max(0, 1 - (ky * distance_green / image.shape[0]))

        #scale PD output by distance falloff to get final steering value
        control_signal_green = (p_term_green + d_term_green) * y_factor_green
        steering_value_green = center - control_signal_green

        #clamp steering to center +/- margin
        steering_value_green = max(center - margin, min(center + margin, steering_value_green))

        #round everything to the nearest whole number
        error_green = round(error_green)
        p_term_green = round(p_term_green)
        d_term_green = round(d_term_green)
        y_error_green = round(y_error_green)
        steering_value_green = round(steering_value_green)

        print(f"Right X: {bottom_right_dot_green[0]}, Left X: {bottom_left_dot_green[0]}, Y Error: {y_error_green}, Y Factor: {round(y_factor_green, 2)}")
        print(f"P: {p_term_green}, D: {d_term_green}, Error: {error_green}, Steering Value: {steering_value_green}")

    #handle whichever blob is closer first (smaller distance = closer to the bottom of the screen = closer to the robot),
    #and only draw that blob's line
    if red_contours and green_contours:
        if distance <= distance_green:
            steering_value = steering_value_red
            cv2.line(image, bottom_right_dot, bottom_left_dot, (0,0,255), 2)
        else:
            steering_value = steering_value_green
            cv2.line(image, bottom_left_dot_green, bottom_right_dot_green, (0,255,0), 2)
    elif red_contours:
        steering_value = steering_value_red
        cv2.line(image, bottom_right_dot, bottom_left_dot, (0,0,255), 2)
    elif green_contours:
        steering_value = steering_value_green
        cv2.line(image, bottom_left_dot_green, bottom_right_dot_green, (0,255,0), 2)
    else:
        #no obstacle visible - fall back to centering between the black walls (STRAIGHT state's wall logic, obstacle_challenge.py)
        wall_frame = Frame(wfx1, wfy1, wfx2, wfy2, image, [lowBlack], [highBlack], frameColor=(255,0,0), leftZoneX=wf_gap_left_x, rightZoneX=wf_gap_right_x, source=image_source)
        left_x, right_x, wall_mask, scan_y1, scan_y2 = wall_frame.getInnerEdgesSplit(color=0, col_threshold=20, contourColor=(255,255,255))

        mid_y = (scan_y1 + scan_y2) // 2
        if left_x is not None:
            cv2.circle(image, (left_x, mid_y), 6, (0,255,255), -1)   # active left boundary
        if right_x is not None:
            cv2.circle(image, (right_x, mid_y), 6, (255,0,255), -1)  # active right boundary
        if (left_x is not None) and (right_x is not None):
            cv2.line(image, (left_x, mid_y), (right_x, mid_y), (0,165,255), 2)

        img_center = (wall_frame.x1 + wall_frame.x2) // 2

        if (left_x is not None) and (right_x is not None):
            corridor_center = (left_x + right_x) // 2
        elif left_x is not None:
            corridor_center = (left_x + wall_frame.x2) // 2
        elif right_x is not None:
            corridor_center = (right_x + wall_frame.x1) // 2
        else:
            corridor_center = img_center

        cv2.circle(image, (int(corridor_center), mid_y), 6, (0,255,0), -1)  # corridor midpoint
        cv2.line(image, (img_center, scan_y1), (img_center, scan_y2), (255,255,255), 1)  # image center reference line

        dz_x1 = img_center - dead_zone_px
        dz_x2 = img_center + dead_zone_px
        dz_y1 = scan_y1 - 10
        dz_y2 = scan_y2 + 10
        cv2.rectangle(image, (dz_x1, dz_y1), (dz_x2, dz_y2), (200,200,200), 1)

        wall_error = corridor_center - img_center
        if abs(wall_error) <= dead_zone_px:
            wall_error = 0

        cv2.line(image, (img_center, mid_y), (int(corridor_center), mid_y), (0,0,255), 2)  # error line

        #PD control for wall-following steering
        derivative_wall = wall_error - previous_error_wall
        control_signal_wall = (kp_wall * wall_error) + (kd_wall * derivative_wall)
        previous_error_wall = wall_error

        steering_value = center + control_signal_wall
        steering_value = max(center - margin, min(center + margin, steering_value))
        steering_value = round(steering_value)

        print(f"Wall Left X: {left_x}, Wall Right X: {right_x}, Corridor Center: {corridor_center}, Img Center: {img_center}, Wall Error: {wall_error}, Steering Value: {steering_value}")

    #send values to micrbit through serial connections
    message = (f"{steering_value} {speed_value} {on} {lines} {indicator}\n")


    #flag large frame-to-frame steering jumps for debugging
    if (abs(steering_value - prev_steer) > 10 or steering_value == 0):
        print(f"*********************************************************************", steering_value)

    prev_steer = steering_value

    # only send if different
    if message != last_message:
        ser.write(message.encode())
        
        ser.flush()

        last_message = message
    time.sleep(0.05)
    #display camera feed after processing
    cv2.imshow("Camera Feed", image)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(f"100 0 1 {lines} OPEN\n".encode())
        time.sleep(0.05)
        break

ser.write(f"85 0 1 {lines} OPEN\n".encode())
print("FINISH")
time.sleep(0.10)
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done