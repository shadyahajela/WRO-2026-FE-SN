#IMPORTS
import sensor.bno055 as bno

import time 
import math
import serial

import numpy as np #where you store hue/color values

from classes.motor_rpm_control import motor, encoder, controller #Required to have close-loop feedback loop control on the DC Motor

from classes.frames import Frame #define your color ranges and other frame-related functions

#get camera working
from picamera2 import Picamera2
import cv2

DRAW = True   # set False for headless runs 

# H = / 2, S = * 2.55, V = * 2.55

#RED BLOCK COLOR VALUES 
# Red wraps around hue 0/180, so two ranges are combined
lowRed1  = np.array([0, 128, 80])
highRed1 = np.array([5, 255, 255])
lowRed2  = np.array([170, 128, 80])
highRed2 = np.array([180, 255, 255])

#   q1BLOCK COLOR VALUES
lowGreen  = np.array([40, 70, 50])
highGreen = np.array([85, 255, 255])

#BLACK WALL COLOR VALUES
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 80])

#MAGENTA WALL COLOR VALUES - counts as wall in wall_frame/corner boxes, except during BETWEEN
lowMagenta  = np.array([155, 100, 80])
highMagenta = np.array([168, 255, 255])

#LAP LINE COLOR VALUES
lowBlue  = np.array([100, 80, 60])
highBlue = np.array([140, 255, 255])

lowOrange  = np.array([6, 100, 75])
highOrange = np.array([35, 255, 255])

DEFAULT_SPEED = 130

#Vehicle mobility control values - send to Arduino
center = 105 # base steering value, PID correction is added/subtracted from this
steering_value = 100 # Calculated steering value to add or subtract from center value
prev_steer = steering_value
steering_margin = 55
speed_value = DEFAULT_SPEED # Speed, 160/135 is lowest, 255 is highest it wa 135
on = 1 # motor direction control: 0 - off, 1 - forward, 2 - reverse

#OBSTACLE AVOIDANCE VALUES - absolute-an gle steering
#find_obstacle_angle_and_draw_lines/calculate_servo_angle_from_obstacle), replacing the old
#corner-position PID which steered off pixel error instead of a real angle
LEFT_OBSTACLE_X_THRESHOLD = 0          # reference point (bottom-left) a red obstacle is steered around
RIGHT_OBSTACLE_X_THRESHOLD = 640   # reference point (bottom-right) a green obstacle is steered around
OBJECT_LINE_ANGLE_THRESHOLD = 45        
OBSTACLE_KP = 1.5
old_obstacle_angle = None      # last computed obstacle angle - reused when the obstacle gets too close to measure reliably
old_obstacle_is_green = None

#WALL FOLLOWING VALUES (used only when no red/green obstacle is visible)
wfx1, wfy1, wfx2, wfy2 = 0, 220, 640, 260   # small band - mirrors obstacle_challenge's no-obstacle default
wf_gap_half_width = 100   # tune this - each gap edge sits this many pixels out from wall_frame's own center (recomputed in wall_follow() off wfx1)
left_wall_offset = 1    # tune - nudges left_x (dynamic, with fallback to wall_frame.x1 when nothing is detected) -
                         # used by CW (hugs the right wall) and by BETWEEN's CCW case 
right_wall_offset = 1   # nudges right_x, used by CCW (hugs the left wall) 
kp_wall = 0.3   
kd_wall = 0.25   
dead_zone_px = 20
previous_error_wall = None   # None sentinel (was 0) - avoids a derivative-kick spike on the first frame after a turn resets it

#CORNER BLACK-DETECTION FRAMES (small boxes stepping diagonally in from each top corner of black_frame)
tr1_x1, tr1_y1, tr1_x2, tr1_y2 = 470, 330, 490, 350   # top-right, outer
tr2_x1, tr2_y1, tr2_x2, tr2_y2 = 490, 310, 510, 330   # top-right, stepped in/down - moved 100px in from the right edge total, no longer touching it

tl1_x1, tl1_y1, tl1_x2, tl1_y2 = 150, 330, 170, 350      # top-left, outer (mirror of tr1)
tl2_x1, tl2_y1, tl2_x2, tl2_y2 = 130, 310, 150, 330    # top-left, stepped in/down (mirror of tr2) - moved 100px in from the left edge total, no longer touching it
CORNER_FULL_PX = 300   # each corner box is 20x20=400px (scaled down from 1550/1600 when boxes were 40x40) - tune on field for how "full" of black counts as a solid wall

#FRONT-CENTER BLACK-DETECTION FRAME - 100x100, centered on the 640x480 feed. Only checked once
#line_detected is True (a lap line has been seen, waiting on turn_delay before entering TURNING);
#if it fills with black, the front wall is right there, so nudge steering toward the upcoming
#turn's direction (CCW turns left, CW turns right) to help it start rotating a beat earlier
FRONT_CENTER_X1, FRONT_CENTER_Y1, FRONT_CENTER_X2, FRONT_CENTER_Y2 = 295, 195, 345, 245
FRONT_CENTER_FULL_PX = 2250   # 50x50=2500px - tune on field for how "full" counts as solid wall
FRONT_CENTER_STEER_NUDGE = 40

CRITICAL_TURN = False
CRITICAL_TURN_TIME = 3.0   # seconds the front-center-box check stays active once CRITICAL_TURN is triggered - tune on field
CRITICAL_TURN_START = None

#PARK TRIGGER BOX (BETWEEN+CCW only) - 20x20, right above wall_frame (wfy1=220), watching for the magenta parking wall
park_box_x1, park_box_y1, park_box_x2, park_box_y2 = 580, 210, 600, 230
PARK_BOX_MAGENTA_PX = 300   # tune on field

#LAP COUNTING / DIRECTION VALUES
CW = 0
CCW = 0
orangeLine = 0
blueLine = 0 
line_detected = False
side_wall_missing = False   # set in STRAIGHT each frame - True when the hugged-side wall drops out of camera view 
start_time_line = time.time()
prev_frame_time = time.time()   # for FPS calculation

LINES_STOP_DELAY = 1.8   # seconds to keep driving once lines reaches 12, before forcing a stop
lines_stop_time = None   # set once, the first frame lines >= 12
parking = None

#TURNING VALUES (without its special-case colored-block handling)
BLOCK_MIN_PIXELS = 600       # contour-area threshold for "obstacle seen" while turning
turn_delay = 0.1           # seconds to wait after a lap line is seen before actually turning
turn_delay_time = None

turn_count = 0
start_time_turn_count = time.time()   # debounce for turn_count, same 1.5s pattern as orangeLine/blueLine's start_time_line

#IMU heading-based turning values - one shared
#kp_turn/kd_turn used everywhere heading-hold steering happens: TURNING, PRE_APPROACH, BETWEEN
TURN_ANGLE_TARGET = 90   # degrees to rotate (IMU heading) before exiting TURNING
target_heading = 0   # absolute target heading for the current turn, set to turn_count*90 (CW) or -turn_count*90 (CCW) on entering TURNING - starts at 0 (not None) so the new unconditional heading_error computation has a target before the first turn

kp_turn = 1.5
kd_turn = 0.45

#derivative-tracking state for PRE_APPROACH's/BETWEEN's target_heading+/-90 turn_error cases
#(their own target differs from target_heading, so they can't share previous_heading_error) -
#reset to None whenever PRE_APPROACH/BETWEEN is freshly entered or the target shifts phase
previous_offset_turn_error = None

#SENSOR FUSION WEIGHTS- blend of IMU heading control vs camera
#wall-following, used for steering in STRAIGHT and TURNING only
IMU_WEIGHT = 0.7
CAM_WEIGHT = 0.3
turn_exit_time = None
SETTLE_TIME = 0.5
previous_heading_error = None

#LEAVE PARKING VALUES (only its "phase 1" drive-straight-out is implemented there)
leave_start_time = None
leave_forward_time = 0.65   # seconds to drive forward out of parking before handing off to normal driving
#0.7

#PARK MANEUVER VALUES (CCW only - CW just stops on entering PARK) - timed sequence: drive
#straight, then reverse while steering right, then reverse while steering left, then stop
park_start_time = None   # set once, the first frame PARK is entered
PARK_STRAIGHT_TIME = 0.6     # seconds to drive straight before reversing 
PARK_REVERSE_RIGHT_TIME = 0.42   # seconds to reverse while steered right 
PARK_REVERSE_LEFT_TIME = 0.5    # seconds to reverse while steered left 


#WAIT/FORCE VALUES (edge case)
EARLY_LINE_Y1 = 250
EARLY_LINE_Y2 = 380   # sits above bottom_frame's own y-range, so the lap line is seen earlier/farther away
EARLY_LINE_MIN_PX = 200   
CENTER_BLACK_FULL_PX = 95   # 10x10 center frame pixel count considered "mostly filled" with black (tune on field)
block_force_turn_time = 0.3   # seconds to force-turn once the center frame fills
block_force_turn_start = None
left_parking = False   # True only for the one-time WAIT/FORCE triggered right as LEAVE finishes - reverses the forcing phase's steering direction once, then resets to False

#STATES
LEAVE = 0      # starting state - drive straight out of parking before normal driving begins
STRAIGHT = 1   # no red/green obstacle visible - wall following
OBSTACLE = 2   # sees a red or green obstacle - avoidance steering
TURNING = 3    # executing a turn at a lap line
BETWEEN = 4    # entered only by finishing a WAIT/FORCE sequence while lines is 11 or 12 - CCW does what STRAIGHT does (blocks treated as red, else wall-follow), CW goes straight to PARK
PARK = 5       # CW after 12 lines - stopjn m
PRE_APPROACH = 6   # drive straight - drive nearly straight until the center frame fills with black, then force a hard turn for a fixed duration before handing back to normal driving
state = LEAVE

STATE_NAMES = {LEAVE: "LEAVE", STRAIGHT: "STRAIGHT", OBSTACLE: "OBSTACLE", TURNING: "TURNING",
               BETWEEN: "BETWEEN", PARK: "PARK", PRE_APPROACH: "PRE_APPROACH"}

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = ""

#getting serial connection working
ser = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
time.sleep(2)

#IMU
bno.initialize() 
initial = bno.get_initial_heading()

picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

time.sleep(3)  # Allow camera to warm up 

picam2.set_controls({"Contrast": 1}) 

#wall-following steering, shared by the STRAIGHT and BETWEEN states (only wall_low/high/idx
#differs between them, computed per-frame in the main loop before this is called)
def wall_follow(image, image_source):
    global previous_error_wall

    #scan gap re-centered on wall_frame's own midpoint every call
    wall_center_x = (wfx1 + wfx2) // 2
    gap_left_x = wall_center_x - wf_gap_half_width
    gap_right_x = wall_center_x + wf_gap_half_width

    wall_frame = Frame(wfx1, wfy1, wfx2, wfy2, image, wall_low, wall_high, frameColor=(255,0,0), leftZoneX=gap_left_x, rightZoneX=gap_right_x, source=image_source)
    left_x, right_x, wall_mask, scan_y1, scan_y2 = wall_frame.getInnerEdgesSplit(color=wall_idx, col_threshold=20, contourColor=(255,255,255))

    if CW or (state == BETWEEN and CCW):
        #CW nudges the left wall position by a fixed offset - uses the REAL detected value when
        #available (stays dynamic to whatever the camera sees), falling back to the frame's own
        #edge only when nothing is detected at all - the offset is always applied either way
        left_x = (left_x if left_x is not None else wall_frame.x1) + left_wall_offset
    elif CCW:
        #CCW: mirror of the above - nudge the right wall position 
        right_x = (right_x if right_x is not None else wall_frame.x2) - right_wall_offset

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

    if previous_error_wall is None:
        previous_error_wall = raw_wall_error

    #PD control for wall-following steering - derivative uses the raw (non-dead-zoned) error, so
    #crossing the dead-zone boundary doesn't itself create a derivative spike 
    derivative_wall = raw_wall_error - previous_error_wall
    control_signal_wall = (kp_wall * wall_error) + (kd_wall * derivative_wall)
    previous_error_wall = raw_wall_error

    #center is not added here - callers combine this raw correction with steering_imu's and
    #add center exactly once, at the very end
    result = max(-steering_margin, min(steering_margin, control_signal_wall))
    result = round(result)

    # print(f"Wall Left X: {left_x}, Wall Right X: {right_x}, Corridor Center: {corridor_center}, Img Center: {img_center}, Wall Error: {wall_error}, Steering Correction: {result}")
    return result, left_x, right_x

#Find the single largest obstacle (red or green) across both contour lists, draw a line from its
#bounding rectangle's near corner to a fixed reference point, and return that line's absolute
#angle plus the obstacle's color and corner 
#ImageAlgorithms.find_obstacle_angle_and_draw_lines(), with its CrashStates handling dropped - Red obstacles
#are steered around using their bounding rectangle's bottom-right corner (passed on the left,
#reference point near the bottom-left of the frame); green obstacles use their bottom-left corner
#(passed on the right, reference point near the bottom-right).
def find_obstacle_angle_and_draw_lines(red_contours, green_contours, image):
    global old_obstacle_angle, old_obstacle_is_green

    candidates = [(c, False) for c in red_contours] + [(c, True) for c in green_contours]
    if not candidates:
        return None, None, None, None

    #pick the biggest blob overall, then check if the second biggest is actually closer (larger
    #bottom-edge y) - same "closer second rectangle wins" rule as the original
    candidates.sort(key=lambda item: cv2.contourArea(item[0]), reverse=True)
    contour, is_green = candidates[0]
    x, y, w, h = cv2.boundingRect(contour)
    y_bottom = y + h

    if len(candidates) > 1:
        contour2, is_green2 = candidates[1]
        x2, y2, w2, h2 = cv2.boundingRect(contour2)
        y_bottom2 = y2 + h2
        if y_bottom2 > y_bottom:
            x, y, w, h, is_green = x2, y2, w2, h2, is_green2
            y_bottom = y_bottom2

    #obstacle is too close to the bottom of the frame to measure reliably
    if y_bottom > image.shape[0] - 60:
        if y_bottom > image.shape[0] - 30:
            return None, None, None, None
        else:
            return old_obstacle_angle, old_obstacle_is_green, x, y_bottom

    if is_green:
        corner = (x, y + h)              # block's own bottom-left corner
        ref_point = (RIGHT_OBSTACLE_X_THRESHOLD, image.shape[0])
        box_color = (0, 255, 0)
    else:
        corner = (x + w, y + h)          # block's own bottom-right corner
        ref_point = (LEFT_OBSTACLE_X_THRESHOLD, image.shape[0])
        box_color = (0, 0, 255)

    cv2.rectangle(image, (x, y), (x + w, y + h), box_color, 1)
    cv2.line(image, corner, ref_point, box_color, 2)

    #absolute angle of the line from the obstacle's corner to the reference point
    rad_angle = math.atan2(corner[1] - image.shape[0], corner[0] - ref_point[0])
    angle = 90 + math.degrees(rad_angle)

    old_obstacle_angle = angle
    old_obstacle_is_green = is_green
    return angle, is_green, corner[0], corner[1]

#Convert an obstacle's absolute angle into a servo steering value 
#ImageAlgorithms.calculate_servo_angle_from_obstacle(), using this file's center/steering_margin - here (per wall_follow's
#corridor-error convention) a higher steering_value steers right, so the correction is added
#instead of subtracted to make red bias left and green bias right as expected.
def calculate_servo_angle_from_obstacle(object_angle, is_green):
    if object_angle is None:
        return None
    if is_green:
        servo_angle = center + (object_angle + OBJECT_LINE_ANGLE_THRESHOLD) * OBSTACLE_KP
    else:
        servo_angle = center + (object_angle - OBJECT_LINE_ANGLE_THRESHOLD) * OBSTACLE_KP
    return max(center - steering_margin, min(center + steering_margin, round(servo_angle)))

#main loop to show camera feed
while True:

    #imu heading
    heading = bno.get_relative_heading(initial)

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
    orangePx = bottom_frame.getContour(1, contourColor=(0, 128, 255))
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0))

    #count orange line when detected (laps)
    if (orangePx > 200) and (orangePx < 2500):
        if time.time() - start_time_line > 2.0:
            orangeLine += 1
            start_time_line = time.time()
            line_detected = True

    #count blue line when detected (laps)
    if (bluePx > 200) and (bluePx < 2500):
        if time.time() - start_time_line > 2.0:
            blueLine += 1
            start_time_line = time.time()
            line_detected = True

    lines = max(orangeLine, blueLine)

    #start the stop timer the first frame lines reaches 12 - LINES_STOP_DELAY seconds later,
    #speed is forced to 0 regardless of state (see the override near message-building below)
    if parking is None:
        if (lines >= 12) and (lines_stop_time is None):
            state = PRE_APPROACH
            parking = True

    #decide CW or CCW from whichever lap-line color is seen first
    if (CW == 0) and (CCW == 0):
        
        if 200 < orangePx < 2500:
            print("Jadoo going clockwise.... woooooo") # Jadoo is the vehicle name if you didnt know
            CW = 1
        elif 200 < bluePx < 2500:
            print("Jadoo going counter-clockwise.... weeeeeee")
            CCW = 1

    #detect red across the entire screen and draw a bounding box around the largest blob
    red_frame = Frame(0, 120, 640, 480, image, [lowRed1, lowRed2], [highRed1, highRed2], frameColor=(0,0,255), source=image_source)
    red_contours = red_frame.getColorContours([0,1], min_area=600)

    #detect green across the entire screen and draw a bounding box around the largest blob
    green_frame = Frame(0, 120, 640, 480, image, [lowGreen], [highGreen], frameColor=(0,255,0), source=image_source)
    green_contours = green_frame.getColorContours(0, min_area=600)

    # #detect green across the entire screen and draw a bounding box around the largest blob
    # green_frame = Frame(0, 120, 640, 480, image, [lowGreen, lowBlue], [highGreen, highBlue], frameColor=(0,255,0), source=image_source)
    # green_contours = green_frame.getColorContours([0,1], min_area=1000)

    obstacle_angle, obstacle_is_green, obstacle_x, obstacle_y = find_obstacle_angle_and_draw_lines(red_contours, green_contours, image)
    steering_value_obstacle = calculate_servo_angle_from_obstacle(obstacle_angle, obstacle_is_green)

    #detect black in the same frame region as the obstacles, draw-only (not used in any calculation)
    black_frame = Frame(0, 120, 640, 480, image, [lowBlack], [highBlack], frameColor=(255,0,0), source=image_source)
    black_contours = black_frame.getColorContours(0, min_area=200)
    cv2.drawContours(image, black_contours, -1, (255,0,0), 2)

    #magenta counts as wall too, except once in BETWEEN (it's the parking marker by then, not terrain)
    wall_low  = [lowBlack] if state == BETWEEN else [lowBlack, lowMagenta]
    wall_high = [highBlack] if state == BETWEEN else [highBlack, highMagenta]
    wall_idx  = 0 if state == BETWEEN else [0, 1]

    #IMU heading-hold signal - target_heading is whatever the early-line-detection logic below
    #last set it to; only read here, never written
    heading_error = ((target_heading - heading + 180) % 360) - 180
    if previous_heading_error is None:
        previous_heading_error = heading_error
    derivative_heading = heading_error - previous_heading_error
    previous_heading_error = heading_error

    #center is not added here - combined with steering_cam and added once, at the very end 
    steering_imu = heading_error * kp_turn + derivative_heading * kd_turn
    print(f"steering imu {round(steering_imu,1)} target heading {target_heading} current heading {round(heading,1)}")

    #four small corner frames, stepping diagonally in from each top corner of black_frame -
    #detect and draw black(+magenta) contours, and keep their pixel counts to check if a side is "full"
    tr1_frame = Frame(tr1_x1, tr1_y1, tr1_x2, tr1_y2, image, wall_low, wall_high, frameColor=(0,255,255), source=image_source)
    tr1_contours = tr1_frame.getColorContours(wall_idx, min_area=10)
    cv2.rectangle(image, (tr1_x1, tr1_y1), (tr1_x2, tr1_y2), (0,255,255), 1)
    cv2.drawContours(image, tr1_contours, -1, (0,255,255), 2)
    tr1Px = sum(cv2.contourArea(c) for c in tr1_contours)

    tr2_frame = Frame(tr2_x1, tr2_y1, tr2_x2, tr2_y2, image, wall_low, wall_high, frameColor=(0,255,255), source=image_source)
    tr2_contours = tr2_frame.getColorContours(wall_idx, min_area=10)
    cv2.rectangle(image, (tr2_x1, tr2_y1), (tr2_x2, tr2_y2), (0,255,255), 1)
    cv2.drawContours(image, tr2_contours, -1, (0,255,255), 2)
    tr2Px = sum(cv2.contourArea(c) for c in tr2_contours)

    tl1_frame = Frame(tl1_x1, tl1_y1, tl1_x2, tl1_y2, image, wall_low, wall_high, frameColor=(0,255,255), source=image_source)
    tl1_contours = tl1_frame.getColorContours(wall_idx, min_area=10)
    cv2.rectangle(image, (tl1_x1, tl1_y1), (tl1_x2, tl1_y2), (0,255,255), 1)
    cv2.drawContours(image, tl1_contours, -1, (0,255,255), 2)
    tl1Px = sum(cv2.contourArea(c) for c in tl1_contours)

    tl2_frame = Frame(tl2_x1, tl2_y1, tl2_x2, tl2_y2, image, wall_low, wall_high, frameColor=(0,255,255), source=image_source)
    tl2_contours = tl2_frame.getColorContours(wall_idx, min_area=10)
    cv2.rectangle(image, (tl2_x1, tl2_y1), (tl2_x2, tl2_y2), (0,255,255), 1)
    cv2.drawContours(image, tl2_contours, -1, (0,255,255), 2)
    tl2Px = sum(cv2.contourArea(c) for c in tl2_contours)

    #total red/green contour area - used by the TURNING state to bail out early if a block shows up mid-turn
    redPx = sum(cv2.contourArea(c) for c in red_contours)
    greenPx = sum(cv2.contourArea(c) for c in green_contours)

    #see the lap line earlier (higher up) than bottom_frame, so there's time to check for the
    #edge case block before actually needing to turn (WAIT/FORCE trigger, below)
    early_line_frame = Frame(270, EARLY_LINE_Y1, 370, EARLY_LINE_Y2, image, [lowBlue, lowOrange], [highBlue, highOrange], frameColor=(0,200,200), source=image_source)
    early_bluePx = early_line_frame.getContour(0, contourColor=(255,85,0))
    early_orangePx = early_line_frame.getContour(1, contourColor=(0,128,255))

    if line_detected and (state != PRE_APPROACH):
        #a lap line was seen AND the hugged-side wall has dropped out of view hold the current steering and wait turn_delay seconds before turning.
        #excludes PRE_APPROACH - this runs before the state dispatch below and would otherwise be
        #able to yank state to TURNING out from under it; PRE_APPROACH may only be exited by its
        #own internal logic (to STRAIGHT or BETWEEN)
        if turn_delay_time is None:
            turn_delay_time = time.time()
        if time.time() - turn_delay_time > turn_delay:
            state = TURNING
            turn_delay_time = None

    # print(IMU_WEIGHT)
    # print(CAM_WEIGHT)
    if state == PARK:
        if CW:
            #CW: timed parking maneuver
            if park_start_time is None:
                park_start_time = time.time()
            park_elapsed = time.time() - park_start_time

            if park_elapsed < PARK_STRAIGHT_TIME:
                speed_value = 130
                steering_value = center
                on = 1
            elif park_elapsed < PARK_STRAIGHT_TIME + PARK_REVERSE_RIGHT_TIME:
                speed_value = 130
                steering_value = center - steering_margin
                on = 2
            elif park_elapsed < PARK_STRAIGHT_TIME + PARK_REVERSE_RIGHT_TIME + PARK_REVERSE_LEFT_TIME:
                speed_value = 130
                steering_value = center + steering_margin
                on = 2
            else:
                speed_value = 0
                steering_value = center
                on = 1
        else:
            #CCW: timed parking maneuver
            if park_start_time is None:
                park_start_time = time.time()
            park_elapsed = time.time() - park_start_time

            if park_elapsed < PARK_STRAIGHT_TIME:
                speed_value = 130
                steering_value = center
                on = 1
            elif park_elapsed < PARK_STRAIGHT_TIME + PARK_REVERSE_RIGHT_TIME:
                speed_value = 130
                steering_value = center + steering_margin
                on = 2
            elif park_elapsed < PARK_STRAIGHT_TIME + PARK_REVERSE_RIGHT_TIME + PARK_REVERSE_LEFT_TIME:
                speed_value = 130
                steering_value = center - steering_margin
                on = 2
            else:
                speed_value = 0
                steering_value = center
                on = 1

    elif state == LEAVE:
        speed_value = 140

        #leaving parking frames (is it CW or CCW?)
        leave_CW_frame = Frame(40, 300, 120, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0,255,0), source=image_source)
        leave_CCW_frame = Frame(520, 300, 600, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0,255,0), source=image_source)

        leave_CW_px = leave_CW_frame.getContour(0, contourColor=(0,200,0)) + leave_CW_frame.getContour(1, contourColor=(0,200,0))
        leave_CCW_px = leave_CCW_frame.getContour(0, contourColor=(0,200,0)) + leave_CCW_frame.getContour(1, contourColor=(0,200,0))

        #decide CW or CCW once, so it doesn't flip mid-maneuver
        if (CW == 0) and (CCW == 0):
            if leave_CW_px > leave_CCW_px:
                CW = 1
            else:
                CCW = 1

        #start the leave timer once, the first time we enter this state
        if leave_start_time is None:
            leave_start_time = time.time()

        elapsed = time.time() - leave_start_time

        if elapsed < leave_forward_time:
            #drive straight out of parking, angled toward the CW/CCW side
            speed_value = 135
            steering_value = center + steering_margin if CW else center - steering_margin
            on = 1
        else:
            #done leaving - check for a block needing WAIT/FORCE treatment before handing off to normal driving
            if CW and (redPx > BLOCK_MIN_PIXELS):
                block_force_turn_start = None
                left_parking = True
                state = OBSTACLE
                CRITICAL_TURN  = True
            elif CCW and (greenPx > BLOCK_MIN_PIXELS):
                block_force_turn_start = None
                left_parking = True
                state = OBSTACLE
                CRITICAL_TURN  = True
            else:
                state = OBSTACLE   # placeholder - redispatched to STRAIGHT/OBSTACLE/BETWEEN next frame
            leave_start_time = None

    elif state == TURNING:
        #if an obstacle shows up mid-turn, bail out back to normal driving/avoidance right away
        #(unchanged - checked first, same as before this port)
        if (redPx > BLOCK_MIN_PIXELS) or (greenPx > BLOCK_MIN_PIXELS):
            state = OBSTACLE   # placeholder - redispatched to STRAIGHT/OBSTACLE/BETWEEN next frame
            line_detected = False
            turn_delay_time = None

        else:
            #PD heading-hold,  - shares kp_turn/kd_turn
            #with PRE_APPROACH/BETWEEN's own PD turn_error handling. Pure IMU, no camera term:
            #wall_follow() only runs in STRAIGHT, so there's no steering_cam to blend in here
            turned_enough = abs(heading_error) <= 35   # loose exit (was <=2) - remainder corrected by STRAIGHT's post-turn settle blend

            if turned_enough: #if the heading has rotated close enough to the target angle, go back to straight mode
                state = STRAIGHT
                line_detected = False
                # frontBlack_detected = False
                side_wall_missing = False
                previous_error_wall = None
                previous_heading_error = None
                turn_delay_time = None
                turn_exit_time = time.time()
                speed_value = DEFAULT_SPEED
                print("EXIT TURN")
            elif abs(heading_error) > 45:
                speed_value = 160
            elif abs(heading_error) > 35:
                speed_value = 140
            else:
                speed_value = 130

            #center added exactly once here - no camera term to blend
            steering_value = center + round(steering_imu)

    elif state == PRE_APPROACH:
        #pure IMU, no camera term - STRAIGHT recomputes its own IMU_WEIGHT/CAM_WEIGHT fresh every
        #frame from turn_exit_time, so this reverts to normal automatically once PRE_APPROACH exits

        #druve straight - keep going nearly straight until the center frame fills with black
        turn_error = (((target_heading - 90 if CW else target_heading + 90)  - heading + 180) % 360) - 180

        if previous_offset_turn_error is None:
            previous_offset_turn_error = turn_error
        derivative_offset_turn_error = turn_error - previous_offset_turn_error
        previous_offset_turn_error = turn_error
        steering_value = center + round(turn_error * kp_turn + derivative_offset_turn_error * kd_turn)
        turned_enough = abs(turn_error) <= 2

        center_frame = Frame(315, 200, 325, 210, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
        centerBlackPx = center_frame.getContour(0, contourColor=(0,255,0))

        if centerBlackPx >= CENTER_BLACK_FULL_PX:
            #switch to the forcing phase, still within WAIT
            block_force_turn_start = time.time()

        if block_force_turn_start is not None:
            #forcing phase: hard turn for a fixed duration, then hand back to normal driving
            #same target as the module-level heading_error - reuse the already PD-damped steering_imu
            turn_error = heading_error
            steering_value = center + round(steering_imu)
            turned_enough = abs(turn_error) <= 2

            if turned_enough: #if the heading has rotated the target angle, go back to straight mode
                state = STRAIGHT
                line_detected = False
                # frontBlack_detected = False
                side_wall_missing = False
                previous_error_wall = None
                previous_heading_error = None
                turn_delay_time = None
                print("EXIT TURN")

            if time.time() - block_force_turn_start > block_force_turn_time:
                #lines has no bearing on BETWEEN anywhere else - this is the ONLY way in: finishing
                #this WAIT/FORCE sequence once lines has reached 12 (it may have ticked over
                #mid-maneuver, since lap-line detection runs every frame regardless of state)
                state = BETWEEN
                #reset, same as the turned_enough->STRAIGHT exit above - otherwise a stale True
                #here would hijack BETWEEN into TURNING on the very next frame (line_detected is
                #no longer guarded once state != PRE_APPROACH)
                line_detected = False
                turn_delay_time = None


    elif state == BETWEEN:
        speed_value = 140
        if CW:
             #CCW: watch the bottom-right corner for the magenta parking wall filling it in
            park_box_frame = Frame(park_box_x1, park_box_y1, park_box_x2, park_box_y2, image, [lowMagenta], [highMagenta], frameColor=(255,0,255), source=image_source)
            park_box_px = park_box_frame.getContour(0, contourColor=(255,0,255))

            if park_box_px >= PARK_BOX_MAGENTA_PX:
                state = PARK
            else:
                #wall_follow's own BETWEEN+CW nudges right_x using right_wall_offset (same
                #dynamic logic CW uses), fused with the module-level IMU heading-hold signal
                #exactly like STRAIGHT does in open_challenge
                right_wall_offset = 50
                steering_cam, left_x, right_x = wall_follow(image, image_source)
                steering_value = center + round(IMU_WEIGHT * steering_imu + CAM_WEIGHT * steering_cam)

        else:
            #CCW: watch the bottom-right corner for the magenta parking wall filling it in
            park_box_frame = Frame(park_box_x1, park_box_y1, park_box_x2, park_box_y2, image, [lowMagenta], [highMagenta], frameColor=(255,0,255), source=image_source)
            park_box_px = park_box_frame.getContour(0, contourColor=(255,0,255))

            if park_box_px >= PARK_BOX_MAGENTA_PX:
                state = PARK
            else:
                #wall_follow's own BETWEEN+CCW nudges left_x using left_wall_offset (same
                #dynamic logic CW uses), fused with the module-level IMU heading-hold signal
                #exactly like STRAIGHT does in open_challenge
                left_wall_offset = 50
                steering_cam, left_x, right_x = wall_follow(image, image_source)
                steering_value = center + round(IMU_WEIGHT * steering_imu + CAM_WEIGHT * steering_cam)

    elif (early_orangePx > EARLY_LINE_MIN_PX) or (early_bluePx > EARLY_LINE_MIN_PX):
        #decide CW/CCW here too, before target_heading needs it below - the early zone sees the
        #first lap line before bottom_frame's own CW/CCW check (further down the frame) ever
        #gets a chance to fire, so without this, target_heading's "if CW" would evaluate on the
        #first turn while CW/CCW are still both 0, and silently fall through to the CCW formula
        #regardless of which color line is actually present
        if (CW == 0) and (CCW == 0):
            if early_orangePx > EARLY_LINE_MIN_PX:
                print("Jadoo going clockwise (early zone)....")
                CW = 1
            elif early_bluePx > EARLY_LINE_MIN_PX:
                print("Jadoo going counter-clockwise (early zone)....")
                CCW = 1

        #approaching a turn - count it, same 1.5s debounce pattern as orangeLine/blueLine
        if time.time() - start_time_turn_count > 2.0:
            turn_count += 1
            start_time_turn_count = time.time()
            target_heading = (turn_count * TURN_ANGLE_TARGET) % 360 if CW else (-turn_count * TURN_ANGLE_TARGET) % 360

        if CW and (early_orangePx > EARLY_LINE_MIN_PX) and (greenPx > BLOCK_MIN_PIXELS):
            #early sighting of a green block near a CW turn - the "edge" case (or, at lines>=12, forced regardless of any block)
            print("wait green")
            block_force_turn_start = None
            CRITICAL_TURN = True

        elif CCW and (early_bluePx > EARLY_LINE_MIN_PX) and (redPx > BLOCK_MIN_PIXELS):
            #early sighting of a red block near a CCW turn - "edge" case (or, at lines>=12, forced regardless of any block)
            print("wait red")
            block_force_turn_start = None
            CRITICAL_TURN = True 
    else:
        turn_delay_time = None

        #decide OBSTACLE vs STRAIGHT fresh every frame based on whether a usable obstacle angle
        #wall-following whenever there's no obstacle angle), then dispatch by state - matches how
        #every other state (PARK/LEAVE/TURNING/PRE_APPROACH/BETWEEN) is identified above
        if steering_value_obstacle is not None:
            state = OBSTACLE
        else:
            state = STRAIGHT

        if state == OBSTACLE:
            steering_value = steering_value_obstacle

        elif state == STRAIGHT:
            #camera wall-following signal - only computed here, since wall_follow() should only
            #run during STRAIGHT 
            steering_cam, left_x, right_x = wall_follow(image, image_source)
            print(f"steering wall {steering_cam}")

            if turn_exit_time is not None:
                elapsed = time.time() - turn_exit_time
                if elapsed < SETTLE_TIME:
                    #post-turn settle: lean on IMU correction briefly to finish closing the
                    #heading gap left by TURNING's loose exit tolerance
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
            steering_value = center + round(IMU_WEIGHT * steering_imu + CAM_WEIGHT * steering_cam)
            print(f"STR STEER {steering_value}")

    #skipped once parked, between, or in PRE_APPROACH
    if state not in (LEAVE, PARK, BETWEEN, PRE_APPROACH):
        if CRITICAL_TURN:
            #front-center box only, for a fixed window after CRITICAL_TURN triggers - if it
            #fills with black, nudge steering toward the turn direction (CW right, CCW left)
            if CRITICAL_TURN_START is None:
                CRITICAL_TURN_START = time.time()

            front_center_frame = Frame(FRONT_CENTER_X1, FRONT_CENTER_Y1, FRONT_CENTER_X2, FRONT_CENTER_Y2, image, [lowBlack], [highBlack], frameColor=(0,255,0), source=image_source)
            front_center_contours = front_center_frame.getColorContours(0, min_area=10)
            cv2.rectangle(image, (FRONT_CENTER_X1, FRONT_CENTER_Y1), (FRONT_CENTER_X2, FRONT_CENTER_Y2), (0,255,0), 1)
            front_center_px = sum(cv2.contourArea(c) for c in front_center_contours)

            if front_center_px >= FRONT_CENTER_FULL_PX:
                #left_parking reverses the nudge direction, same as it does for PRE_APPROACH's
                #forcing-phase turn_error - leaving parking already points the robot toward the
                #first turn, so the usual CW-right/CCW-left nudge would be backwards here
                if left_parking:
                    if CW:
                        steering_value -= FRONT_CENTER_STEER_NUDGE
                    elif CCW:
                        steering_value += FRONT_CENTER_STEER_NUDGE
                else:
                    if CW:
                        steering_value += FRONT_CENTER_STEER_NUDGE
                    elif CCW:
                        steering_value -= FRONT_CENTER_STEER_NUDGE

            if time.time() - CRITICAL_TURN_START > CRITICAL_TURN_TIME:
                CRITICAL_TURN = False
                CRITICAL_TURN_START = None
                left_parking = False

        #if a corner frame is solidly black ("full"), nudge steering away from that side
        right_full = (tr1Px >= CORNER_FULL_PX) and (tr2Px >= CORNER_FULL_PX)
        left_full = (tl1Px >= CORNER_FULL_PX) and (tl2Px >= CORNER_FULL_PX)
        if right_full:
            steering_value -= 15
            print("             ***** Nudge Right More")
        elif (tr1Px >= CORNER_FULL_PX) or (tr2Px >= CORNER_FULL_PX):
            steering_value -= 10
            print("             ***** Nudge Right")
        if left_full:
            steering_value += 15
            print("             ***** Nudge Left More")
        elif (tl1Px >= CORNER_FULL_PX) or (tl2Px >= CORNER_FULL_PX):
            steering_value += 10
            print("             ***** Nudge Left")

        steering_value = max(center - steering_margin, min(center + steering_margin, steering_value))

    #once lines has been >= 12 for LINES_STOP_DELAY seconds, force a stop - except in PARK, which
    #runs its own timed maneuver (CCW) or stop (CW) and must not be stomped on every frame
    if (state != PARK) and (lines_stop_time is not None) and (time.time() - lines_stop_time > LINES_STOP_DELAY):
        speed_value = 0
        steering_value = center

    #FPS calculation - time elapsed since the previous frame
    current_frame_time = time.time()
    fps = 1 / (current_frame_time - prev_frame_time) if current_frame_time != prev_frame_time else 0
    prev_frame_time = current_frame_time

    print(f"state: {STATE_NAMES[state]} lines: {lines} steering: {steering_value} fps: {fps:.1f}")

    if DRAW:
        #print values on camera feed
        cv2.putText(image, f"Lines: {lines}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Direction: {'CW' if CW else 'CCW'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"State: {STATE_NAMES[state]}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Heading: {heading}", (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Target Heading: {target_heading}", (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"FPS: {fps:.1f}", (400, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)



    #send values to micrbit through serial connections
    message = (f"$ {steering_value} {speed_value} {on}\n")

    #flag large frame-to-frame steering jumps for debugging
    if (abs(steering_value - prev_steer) > 10 or steering_value == 0):
        print(f"*********************************************************************", steering_value)

    prev_steer = steering_value

    # only send if different
    if message != last_message:
        print(message)
        ser.write(message.encode())
        ser.flush()
        time.sleep(0.03)  
        last_message = message
                                                                                                                           
    if DRAW:
        #display camera feed after processing
        cv2.imshow("Camera Feed", image)

        # Check for 'q' key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            ser.write(f"$ {center} 0 1\n".encode())
            ser.flush()
            time.sleep(0.05)
            break

ser.write(f"$ {center} 0 1\n".encode())
ser.flush()
print("FINISH")
time.sleep(0.10)
bno.cleanup()
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done