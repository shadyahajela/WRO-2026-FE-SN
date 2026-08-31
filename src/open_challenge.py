#IMPORTS
import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # ensure src/ is on the path so `sensor` resolves regardless of how this script is launched
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

# IMU
bno.initialize() 
initial = bno.get_initial_heading()

#What direction are we going? CW or CCW?
CW = 0
CCW = 0

line_count = 12 # Number of lines to be counted
orangeLine = 0
blueLine = 0
lines = 0 # number of lines counted
line_detected = False
frontBlack_detected = False
side_wall_missing = False

#COLOR VALUES
# H = / 2, S = * 2.55, V = * 2.55
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 100])

lowBlue  = np.array([100, 70, 50])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([2, 70, 50])
highOrange = np.array([35, 255, 255])

#Things I added
TURN_LINE_THRESHOLD = 200
COUNT_LINE_THRESHOLD = 600

turn_exit_time = None
SETTLE_TIME = 0.5

previous_error_wall = None
previous_heading_error = None


#Vehicle mobility control values - send to Arduino
center = 105 #center value for steering, adjust as needed (was 85)
steering_margin = 40 
steering_value = center # Calculated steering value to add or subtract from center value
speed_value = 200 # Speed, 160 is lowest, 255 is highest
on = 1 # motor direction control: 0 - off, 1 - forward, 2 - reverse

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = "" 

#WALL FOLLOWING VALUES (ported from obstaclev2.py's wall_follow(), used by STRAIGHT)
wfx1, wfy1, wfx2, wfy2 =  0, 220, 640, 260

wf_gap_half_width = 50   # tune this - each gap edge sits this many pixels out from wall_frame's own center
wall_offset_cw = 0   # tune - CW hugs the right wall, so left_x is faked at wall_frame.x1 + this; needs to be pushed further in than CCW's to bias corridor_center right enough (110 wasn't enough - car swung away from the right wall instead of hugging it)
wall_offset_ccw = 0   # tune - CCW hugs the left wall, so right_x is faked at wall_frame.x2 - this; already working ~90% of the time at this value
kp_wall = 0.3   # lowered from 0.22 - still swinging past center once before settling, so the P term was too punchy for the added damping to fully absorb
kd_wall = 0.25   # raised from 0.15 - previous bump cut the zigzag but wasn't enough damping to stop the single overshoot
kp_turn = 0.5   # lowered from 0.8
kd_turn = 0.45   # raised from 0.3 - same reasoning as kd_wall, for the heading-hold term
dead_zone_px = 20

#SENSOR FUSION WEIGHTS - blend of IMU heading control vs camera wall-following, used for steering in both states
IMU_WEIGHT = 0.7
CAM_WEIGHT = 0.3

#timer variables
start_time_line = time.time()
start_time_finshed = 0
prev_frame_time = time.time()   # for FPS calculation

turn_delay = 0.2  # seconds to wait before confirming a turn
TURN_ANGLE_TARGET = 90   # degrees to rotate (IMU heading) before exiting TURNING

turn_delay_time = None
turn_count = 0   # increments every time TURNING is entered - target_heading is always turn_count*90 absolute, so turns snap to true cardinal headings instead of drifting off each other
target_heading = 0   # cardinal heading to hold - starts at 0 (initial heading) so IMU fusion has a target before the first turn

#STATES
STRAIGHT = 0
TURNING = 1

STATE_NAMES = {STRAIGHT: "STRAIGHT", TURNING: "TURNING"}

state = STRAIGHT

#wall-following steering, ported from obstaclev2.py's STRAIGHT state (no block detection here)
def wall_follow(image):
    global previous_error_wall

    #scan gap re-centered on wall_frame's own midpoint every call
    wall_center_x = (wfx1 + wfx2) // 2
    gap_left_x = wall_center_x - wf_gap_half_width
    gap_right_x = wall_center_x + wf_gap_half_width

    wall_frame = Frame(wfx1, wfy1, wfx2, wfy2, image, [lowBlack], [highBlack], frameColor=(255,0,0), leftZoneX=gap_left_x, rightZoneX=gap_right_x)
    left_x, right_x, wall_mask, scan_y1, scan_y2 = wall_frame.getInnerEdgesSplit(color=0, col_threshold=20, contourColor=(255,255,255))

    if CW:
        #CW: pretend there's a wall at a fixed offset from wall_frame's own x1, regardless of what's actually detected
        left_x = wall_frame.x1 + wall_offset_cw
    elif CCW:
        #CCW: mirror of the above - fake wall offset in from wall_frame's own x2
        right_x = wall_frame.x2 - wall_offset_ccw

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

    raw_wall_error = corridor_center - img_center
    wall_error = raw_wall_error
    if abs(wall_error) <= dead_zone_px:
        wall_error = 0

    cv2.line(image, (img_center, mid_y), (int(corridor_center), mid_y), (0,0,255), 2)  # error line

    # If this is the first frame after a turn,
    # don't create an artificial derivative spike.
    if previous_error_wall is None:
        previous_error_wall = raw_wall_error

    #PD control for wall-following steering - derivative uses the raw (non-dead-zoned) error, so
    #crossing the dead-zone boundary doesn't itself create a derivative spike
    derivative_wall = raw_wall_error - previous_error_wall
    control_signal_wall = (kp_wall * wall_error) + (kd_wall * derivative_wall)
    previous_error_wall = raw_wall_error

    #center is not added here - callers combine this raw correction with steering_imu's and
    #add center exactly once, at the very end
    steer = max(-steering_margin, min(steering_margin, control_signal_wall))
    steer = round(steer)

    print(f"Wall Left X: {left_x}, Wall Right X: {right_x}, Corridor Center: {corridor_center}, Img Center: {img_center}, Wall Error: {wall_error}, Steering Correction: {steer}")
    return steer, left_x, right_x

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


    if orangePx > TURN_LINE_THRESHOLD or bluePx > TURN_LINE_THRESHOLD:
        line_detected = True

    # LINE COUNTING
    if orangePx > 600:
        if time.time() - start_time_line > 1.2:#if orange line is detected for more than 1 second, count it
            orangeLine += 1
            start_time_line = time.time()

    # LINE COUNTING
    if bluePx > 600:
        if time.time() - start_time_line > 1.2:#if blue line is detected for more than 1 second, count it
            blueLine += 1
            start_time_line = time.time()#reset timer when blue line is detected

    lines = max(orangeLine, blueLine)

    #CW OR CCW?
    if (CW == 0) and (CCW == 0):
        if 200 < orangePx < 2500:
            print("cw")
            CW = 1
        elif 200 < bluePx < 2500:
            print("ccw")
            CCW = 1

    #camera wall-following signal - computed every frame (feeds both the fusion below and the
    #side-wall-gap turn-entry check), so it always reflects this frame's fresh left_x/right_x
    steering_cam, left_x, right_x = wall_follow(image)

    if state == STRAIGHT:
        if turn_exit_time is not None:

            elapsed = time.time() - turn_exit_time

            if elapsed < SETTLE_TIME:
                IMU_WEIGHT = 0.4
                CAM_WEIGHT = 0.6
            else:
                turn_exit_time = None
                IMU_WEIGHT = 0.2
                CAM_WEIGHT = 0.8

        else:
            IMU_WEIGHT = 0.2
            CAM_WEIGHT = 0.8

        if (CW == 1) and (CCW == 0):
            side_wall_missing = right_x is None
        elif (CW == 0) and (CCW == 1):
            side_wall_missing = left_x is None

        #check if it is turning
        if line_detected:
            if turn_delay_time is None:
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
            turn_delay_time = None

    #IMU heading-hold signal - target_heading is the last completed turn's cardinal heading while
    #STRAIGHT (so this just corrects drift), or the new turn's cardinal heading while TURNING
    heading_error = ((target_heading - heading + 180) % 360) - 180

    #PD, not P-only - undamped P control overshot target_heading and drove the zigzag seen across
    #all three wall-distance tests (this term runs regardless of wall proximity)
    if previous_heading_error is None:
        previous_heading_error = heading_error
    derivative_heading = heading_error - previous_heading_error
    previous_heading_error = heading_error

    #center is not added here - combined with steering_cam and added once, at the very end
    steering_imu = heading_error * kp_turn + derivative_heading * kd_turn
    print(f"steering imu {steering_imu} target heading {target_heading} current heading {heading}")

    

    #------------------------------------------------------------------------------------------------------
    #if the state is turning
    if state == TURNING:
        IMU_WEIGHT = 0.8
        CAM_WEIGHT = 0.2
        turned_enough = abs(heading_error) <= 35

        if turned_enough: #if the heading has rotated the target angle, go back to straight mode
            state = STRAIGHT
            line_detected = False
            turn_exit_time = time.time()
            # frontBlack_detected = False
            side_wall_missing = False
            previous_error_wall = None
            previous_heading_error = None
            turn_delay_time = None
            speed_value = 200
            print("EXIT TURN")
        elif abs(heading_error) > 45:
            speed_value = 160
        elif abs(heading_error) > 35:
            speed_value = 140
        else:
            speed_value = 130
    

    #weighted sensor fusion: 70% IMU heading, 30% camera wall-following - used for both states
    #center is added exactly once here, after combining the two raw corrections
    steering_value = center + round(IMU_WEIGHT * steering_imu + CAM_WEIGHT * steering_cam)
    print(f"steering: {steering_value}")
    #stop when see all the orange line
    if lines >=  line_count:
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
    
    #FPS calculation - time elapsed since the previous frame
    current_frame_time = time.time()
    fps = 1 / (current_frame_time - prev_frame_time) if current_frame_time != prev_frame_time else 0
    prev_frame_time = current_frame_time

    #print values on camera feed
    cv2.putText(image, f"Lines: {lines}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Direction: {'CW' if CW else 'CCW'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"State: {'turn' if state else 'straight'}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Heading: {heading}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Target Heading: {target_heading}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"FPS: {fps:.1f}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)



    message = (f"$ {steering_value} {speed_value} {on}\n")

    # only send if different
    if message != last_message:
    # if True:
        ser.write(message.encode())
        ser.flush()
        print(f"************* {message}")

        last_message = message

    time.sleep(0.01)

    #display camera feed after processing
    cv2.imshow("Camera Feed", image)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(f"$ 100 0 1\n".encode())
        time.sleep(0.05)
        break

ser.write(f"$ 100 0 1\n".encode())
print("FINISH")
time.sleep(0.10)
bno.cleanup()
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done