#IMPORTS
import time 
import math
import serial

import numpy as np #where you store hue/color values

from frames import Frame #define your color ranges and other frame-related functions

#get camera working
from picamera2 import Picamera2
import cv2

#COLOR VALUES
# H - /2, S - x 2.55, V - x 2.55

#RED COLOR VALUES (from obstacle_FAKE.py)
# Red wraps around hue 0/180, so two ranges are combined
lowRed1  = np.array([0, 128, 80])
highRed1 = np.array([5, 255, 255])
lowRed2  = np.array([170, 128, 80])
highRed2 = np.array([180, 255, 255])

#GREEN COLOR VALUES (from obstacle_challenge.py)
lowGreen  = np.array([40, 70, 50])
highGreen = np.array([85, 255, 255])

#BLACK WALL COLOR VALUES (from obstacle_challenge.py)
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 80])

#LAP LINE COLOR VALUES (from obstacle_challenge.py)
lowBlue  = np.array([100, 110, 100])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([6, 100, 75])
highOrange = np.array([35, 255, 255])

#MICROBIT VALUES
center = 100 # base steering value, PID correction is added/subtracted from this
steering_value = 100 # Calculated steering value to add or subtract from center value
prev_steer = steering_value
margin = 55
speed_value = 180 # Speed, 160/135 is lowest, 255 is highest
on = 1

lines = 0

#PID VALUES
kp = 0.09
kd = 0.2
ky = 1.0  # tune how aggressively distance affects steering: >1 = harsher falloff (goes light faster with distance), <1 = gentler falloff
previous_error = 0
previous_error_green = 0

#WALL FOLLOWING VALUES (used only when no red/green obstacle is visible)
wfx1, wfy1, wfx2, wfy2 = 0, 220, 640, 260   # small band - mirrors obstacle_challenge.py's no-obstacle default
wf_gap_left_x = 220
wf_gap_right_x = 420
kp_wall = 0.3
kd_wall = 0.22
dead_zone_px = 20
previous_error_wall = 0

#CORNER BLACK-DETECTION FRAMES (small boxes stepping diagonally in from each top corner of black_frame)
tr1_x1, tr1_y1, tr1_x2, tr1_y2 = 560, 400, 600, 440   # top-right, outer
tr2_x1, tr2_y1, tr2_x2, tr2_y2 = 600, 360, 640, 400   # top-right, stepped in/down

tl1_x1, tl1_y1, tl1_x2, tl1_y2 = 40, 400, 80, 440      # top-left, outer (mirror of tr1)
tl2_x1, tl2_y1, tl2_x2, tl2_y2 = 0, 360, 40, 400    # top-left, stepped in/down (mirror of tr2)
CORNER_FULL_PX = 1550   # each corner box is 40x40=1600px - tune on field for how "full" of black counts as a solid wall

#LAP COUNTING / DIRECTION VALUES (from obstacle_challenge.py)
CW = 0
CCW = 0
orangeLine = 0
blueLine = 0
line_detected = False
start_time_line = time.time()

#TURNING VALUES (from obstacle_challenge.py, without its special-case colored-block handling)
OBSTACLE_TURN = 200        # contour-area threshold for "obstacle seen" while turning
turn_delay = 0.4           # seconds to wait after a lap line is seen before actually turning
turn_execution_time = 0.4  # seconds to hold the turn before handing back to normal driving
turn_delay_time = None
turning_start_time = 0

#STATES
STRAIGHT = 0
TURNING = 1
state = STRAIGHT

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

    #bottom line detection frame (blue/orange lap lines) - also used to decide CW vs CCW
    bottom_frame = Frame(295, 405, 345, 455, image, [lowBlue, lowOrange], [highBlue, highOrange], source=image_source)
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0))
    orangePx = bottom_frame.getContour(1, contourColor=(0, 128, 255))

    #count orange line when detected (laps)
    if (orangePx > 200) and (orangePx < 2500):
        if time.time() - start_time_line > 1.5:
            orangeLine += 1
            start_time_line = time.time()
            line_detected = True

    #count blue line when detected (laps)
    if (bluePx > 200) and (bluePx < 2500):
        if time.time() - start_time_line > 1.5:
            blueLine += 1
            start_time_line = time.time()
            line_detected = True

    lines = max(orangeLine, blueLine)

    #decide CW or CCW from whichever lap-line color is seen first
    if (CW == 0) and (CCW == 0):
        if 500 < orangePx < 10000:
            CW = 1
        elif 500 < bluePx < 10000:
            CCW = 1

    #detect red across the entire screen and draw a bounding box around the largest blob
    red_frame = Frame(0, 120, 640, 480, image, [lowRed1, lowRed2], [highRed1, highRed2], frameColor=(0,0,255), source=image_source)
    red_contours = red_frame.getColorContours([0,1], min_area=1000)

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
    green_frame = Frame(0, 120, 640, 480, image, [lowGreen], [highGreen], frameColor=(0,255,0), source=image_source)
    green_contours = green_frame.getColorContours(0, min_area=1000)

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
        y_factor_green = max(0, 1 - (ky * distance_green / image.shape[0]))   # 1.0 = touching the bottom (closest), 0.0 = at/past the falloff point set by ky
    

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

    #detect black in the same frame region as the obstacles, draw-only (not used in any calculation)
    black_frame = Frame(0, 120, 640, 480, image, [lowBlack], [highBlack], frameColor=(255,0,0), source=image_source)
    black_contours = black_frame.getColorContours(0, min_area=200)
    cv2.drawContours(image, black_contours, -1, (255,0,0), 2)

    #four small corner frames, stepping diagonally in from each top corner of black_frame -
    #detect and draw black contours, and keep their pixel counts to check if a side is "full"
    tr1_frame = Frame(tr1_x1, tr1_y1, tr1_x2, tr1_y2, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
    tr1Px = tr1_frame.getContour(0, contourColor=(0,255,255))

    tr2_frame = Frame(tr2_x1, tr2_y1, tr2_x2, tr2_y2, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
    tr2Px = tr2_frame.getContour(0, contourColor=(0,255,255))

    tl1_frame = Frame(tl1_x1, tl1_y1, tl1_x2, tl1_y2, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
    tl1Px = tl1_frame.getContour(0, contourColor=(0,255,255))

    tl2_frame = Frame(tl2_x1, tl2_y1, tl2_x2, tl2_y2, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
    tl2Px = tl2_frame.getContour(0, contourColor=(0,255,255))

    #total red/green contour area - used by the TURNING state to bail out early if a block shows up mid-turn
    redPx = sum(cv2.contourArea(c) for c in red_contours)
    greenPx = sum(cv2.contourArea(c) for c in green_contours)

    if state == STRAIGHT:
        if line_detected:
            #a lap line was seen - hold the current steering and wait turn_delay seconds before actually turning
            if turn_delay_time is None:
                turn_delay_time = time.time()
            if time.time() - turn_delay_time > turn_delay:
                state = TURNING
                turning_start_time = time.time()
                turn_delay_time = None
        else:
            turn_delay_time = None

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

    elif state == TURNING:
        if CW:
            steering_value = center + margin
        elif CCW:
            steering_value = center - margin

        #if an obstacle shows up mid-turn, bail out back to normal driving/avoidance right away
        if (redPx > OBSTACLE_TURN) or (greenPx > OBSTACLE_TURN):
            state = STRAIGHT
            line_detected = False
            turn_delay_time = None
        elif time.time() - turning_start_time > turn_execution_time:
            state = STRAIGHT
            line_detected = False
            turn_delay_time = None

    #if a corner frame is solidly black ("full"), nudge steering away from that side
    right_full = (tr1Px >= CORNER_FULL_PX) and (tr2Px >= CORNER_FULL_PX)
    left_full = (tl1Px >= CORNER_FULL_PX) and (tl2Px >= CORNER_FULL_PX)

    if right_full:
        steering_value -= 20
    if left_full:
        steering_value += 20

    steering_value = max(center - margin, min(center + margin, steering_value))

    indicator = state

    #send values to micrbit through serial connections
    message = (f"{steering_value} {speed_value} {on} {lines} {indicator}\n")


    #flag large frame-to-frame steering jumps for debugging
    if (abs(steering_value - prev_steer) > 10 or steering_value == 0):
        print(f"*********************************************************************", steering_value)

    prev_steer = steering_value

    # only send if different
    if True: #message != last_message:
        print(message)
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