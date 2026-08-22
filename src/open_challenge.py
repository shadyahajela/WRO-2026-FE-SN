#IMPORTS
import sensor.bno055 as bno

import time 

import serial

import numpy as np #where you store hue/color values

from frames import Frame #define your color ranges and other frame-related functions

import math

#get camera working 
from picamera2 import Picamera2
import cv2
picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (640, 480)}
)

picam2.configure(config)
picam2.start()

time.sleep(2)  # Allow camera to warm up 

#getting serial connection working
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
time.sleep(2)

#IMU
bno.initialize() 
initial = bno.get_initial_heading()

#What direction are we going? CW or CCW?
CW = 0
CCW = 0

orangeLine = 0
blueLine = 0
lines = 0
line_detected = False
frontBlack_detected = False
side_wall_missing = False


#COLOR VALUES
# H - /2, S - x 2.55, V - x 2.55
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 100])

lowBlue  = np.array([100, 70, 50])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([5, 100, 75])
highOrange = np.array([35, 255, 255])

#MICROBIT VALUES
steering_value = 0 # Calculated steering value to add or subtract from center value
center = 100 #center value for steering, adjust as needed (was 85)
steering_margin = 40 
speed_value = 255 # Speed, 160 is lowest, 255 is highest
on = 1

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = ""

#WALL FOLLOWING VALUES (ported from obstaclev2.py's wall_follow(), used by STRAIGHT)
wfx1, wfy1, wfx2, wfy2 =  0, 220, 640, 260
wf_gap_half_width = 100   # tune this - each gap edge sits this many pixels out from wall_frame's own center
kp_wall = 0.15
kd_wall = 0.2
kp_turn = 1
dead_zone_px = 20
previous_error_wall = 0

#timer variables
start_time_line = time.time()
start_time_finshed = 0

turn_delay = 0.15  # seconds to wait before confirming a turn
TURN_ANGLE_TARGET = 90   # degrees to rotate (IMU heading) before exiting TURNING
# HEADING_TURN_SIGN = 1    # flip to -1 on the field if CW/CCW come out reversed for this IMU's mounting

turn_delay_time = None
turning_start_heading = None
turn_count = 0   # increments every time TURNING is entered - target_heading is always turn_count*90 absolute, so turns snap to true cardinal headings instead of drifting off each other
target_heading = None

#STATES
STRAIGHT = 0
TURNING = 1

state = STRAIGHT
indicator = "STRA"

#wall-following steering, ported from obstaclev2.py's STRAIGHT state (no block detection here)
def wall_follow(image):
    global previous_error_wall

    #scan gap re-centered on wall_frame's own midpoint every call
    wall_center_x = (wfx1 + wfx2) // 2
    gap_left_x = wall_center_x - wf_gap_half_width
    gap_right_x = wall_center_x + wf_gap_half_width

    wall_frame = Frame(wfx1, wfy1, wfx2, wfy2, image, [lowBlack], [highBlack], frameColor=(255,0,0), leftZoneX=gap_left_x, rightZoneX=gap_right_x)
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

    result = center + control_signal_wall
    result = max(center - steering_margin, min(center + steering_margin, result))
    result = round(result)

    print(f"Wall Left X: {left_x}, Wall Right X: {right_x}, Corridor Center: {corridor_center}, Img Center: {img_center}, Wall Error: {wall_error}, Steering Value: {result}")
    return result, left_x, right_x

#main loop to show camera feed
while True:

    #imu heading
    heading = bno.get_relative_heading(initial)

    #camera feed
    image = picam2.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    # flip upside down
    image = cv2.rotate(image, cv2.ROTATE_180)
    

    #create frames
    # keep existing bottom color checks for lap/dir detection
    bottom_frame = Frame(295, 405, 345, 455, image, [lowBlue, lowOrange], [highBlue,  highOrange])
    orangePx = bottom_frame.getContour(1, contourColor =(0, 128, 255)) #orange
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0)) #blue

    # single wall scan for the whole frame (steering + side_wall_missing) - skipped while
    # turning so it can't influence steering/state decisions
    if state != TURNING:
        wall_steering, left_x, right_x = wall_follow(image)
    else:
        wall_steering, left_x, right_x = None, None, None


    #count orange line when detected (laps)
    if (orangePx > 200) and (orangePx < 2500):
        if time.time() - start_time_line > 1.5: #if orange line is detected for more than 1 second, count it
            orangeLine += 1
            start_time_line = time.time() #reset timer when orange line is detected
            line_detected = True

    #count BLUE line when detected (laps)
    if (bluePx > 200) and (bluePx < 2500):
        if time.time() - start_time_line > 1.5: #if blue line is detected for more than 1 second, count it
            blueLine += 1
            start_time_line = time.time() #reset timer when blue line is detected
            line_detected = True

    lines = max(orangeLine, blueLine)

    #CW OR CCW?
    if (CW == 0) and (CCW == 0):
        if 200 < orangePx < 2500:
            print("cw")
            CW = 1
        elif 200 < bluePx < 2500:
            print("ccw")
            CCW = 1
    elif state != TURNING and (CW == 1) and (CCW == 0):
        side_wall_missing = right_x is None
    elif state != TURNING and (CW == 0) and (CCW == 1):
        side_wall_missing = left_x is None

    #to see if we will turn soon (black frame, if it saw enough of the wall)
    # if (frontBlack > 2000):
    #     frontBlack_detected = True

    if state == STRAIGHT:
        #check if it is turning
        if line_detected and side_wall_missing:
            if turn_delay_time is None:
                print("Here")
                turn_delay_time = time.time()

            if time.time() - turn_delay_time > turn_delay:
                state = TURNING
                turn_count += 1
                #absolute cardinal target (turn_count*90), not relative to turning_start_heading -
                #so turns snap to true 90/180/270/0 instead of drifting off each other's small errors
                target_heading = (turn_count * TURN_ANGLE_TARGET) % 360 if CW else (-turn_count * TURN_ANGLE_TARGET) % 360
                turn_delay_time = None  # reset for next detection
                print("ENTER TURN")
        else:
            # Reset if the condition is no longer true
            turn_delay_time = None
            steering_value = wall_steering

    #------------------------------------------------------------------------------------------------------
    #if the state is turning
    elif state == TURNING:
        turn_error = ((target_heading - heading + 180) % 360) - 180
        steering_value = turn_error * kp_turn
        turned_enough = abs(turn_error) <= 2

        if turned_enough: #if the heading has rotated the target angle, go back to straight mode
            state = STRAIGHT
            line_detected = False
            # frontBlack_detected = False
            side_wall_missing = False
            previous_error_wall = 0
            turn_delay_time = None
            print("EXIT TURN")
    

    #stop when see all the orange line
    if lines >=  12:
        if start_time_finshed == 0:
            start_time_finshed = time.time() #start timer when all orange lines are detected
        if time.time() - start_time_finshed > 5: #if all orange lines are detected for more than 3 seconds, stop the car
            speed_value = 0 


    # Hello Shadyta, this is your coach suffering with the servo
    #Straight angle is 85 dont question why :)
    #max andgles will be 135 and 55 

    #120
    if steering_value > (center + steering_margin):
        steering_value = (center + steering_margin)

    #40
    elif steering_value < (center - steering_margin):
        steering_value = (center - steering_margin)
    
    #print values on camera feed
    cv2.putText(image, f"Lap: {math.ceil((lines/4))}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Direction: {'CW' if CW else 'CCW'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"State: {state}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Heading: {heading}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    #send values to micrbit through serial connections
    print(f"Steering: {steering_value}, Speed: {speed_value}, ON?: {on}, LINES: {lines}, STATE: {state}")
    print(f"Line Detected: {line_detected}, Empty Wall: {side_wall_missing}, cw: {CW}, ccw: {CCW}")
    print(f"Heading: {heading}, Turning Start Heading: {heading}\n")

    indicator = state

    message = (f"{steering_value} {speed_value} {on} {lines} {indicator}\n")

    # only send if different
    if True: #message != last_message:
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
bno.cleanup()
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done
