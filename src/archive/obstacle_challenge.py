#IMPORTS
import time 
import math
import serial

import numpy as np #where you store hue/color values

from classes.frames import Frame #define your color ranges and other frame-related functions

#get camera working 
from picamera2 import Picamera2
import cv2

DRAW = True

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
# lowBlack  = np.array([0, 0, 0])
# highBlack = np.array([180, 255, 100])
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180,255, 80])

lowBlue  = np.array([100, 110, 100])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([6, 100, 75])
highOrange = np.array([35, 255, 255])

#OBSTACLE CHALLENGE COLOR VALUES (starting points - tune on field/lighting)
# Red wraps around hue 0/180, so two ranges are combined (mirrors Frame.getContour's isRed pattern)
# lowRed1  = np.array([0, 185, 150])
# highRed1 = np.array([4, 255, 255])
# lowRed2  = np.array([165, 100, 60])
# highRed2 = np.array([180, 255, 255])
 
lowRed1  = np.array([0, 128, 50])
highRed1 = np.array([5, 255, 255])
lowRed2  = np.array([170, 128, 50])
highRed2 = np.array([180, 255, 255])
        
lowGreen  = np.array([40, 80, 60])
highGreen = np.array([80, 255, 255])

# lowMagenta  = np.array([140, 70, 50])
# highMagenta = np.array([165, 220, 220])
lowMagenta  = np.array([155, 100, 50])
highMagenta = np.array([168, 255, 255])

#MICROBIT VALUES
steering_value = 0 # Calculated steering value to add or subtract from center value
center = 100 #center value for steering, adjust as needed (was 85) - matches open_challenge.py's tuned straight angle
steering_margin = 55
speed_value = 160 # Speed, 160/135 is lowest, 255 is highest
on = 1

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = ""

#wall error kp steering, kp - 0.54, 0.52
kp = 0.32
kd = 0.22
#0.22
previous_error = 0 

alpha = 0.5 # slightly stronger smoothing
max_steering_correction = 40

# Lane estimation for partial visibility handling
track_width = 240

#estimated_lane_width = 240  # initial guess in pixels (tweakable)
#lane_width_alpha = 0.05      # smoothing factor for running average

last_corridor_center = None
last_detection_time = 0
filtered_center = None
corridor_center = None
steering_error = 0

obstacle_y_alpha = 0.3  # smoothing for the obstacle's row (mid_y) - lower = less flicker, more lag
filtered_obstacle_y = None

#timer variables
start_time_line = time.time()
start_time_finshed = 0

turn_delay = 0.4  # seconds to wait before confirming a turn 0.5
turn_execution_time = 0.4  # seconds to execute the turn 255 - 0.15. 160/135 - 0.4

turn_delay_time = None
turning_start_time = 0

CENTER_BLACK_FULL_PX = 95   # 100x100 center frame pixel count considered "mostly filled" with black (tune on field)
block_force_turn_time = 2.0   # seconds to force-turn once the center frame fills
block_force_turn_start = None

#exit parkin
leave_start_time = None
leave_forward_time = 0.7      # seconds to drive forward before turning out of parking 0.3
leave_turn_time = 0.3  #
leave_turn_time2 = 0.4#
leave_turn_time3 = 0.68
leave_turn_time4 = 0.4

#enter parkings
saw_turn = None

#OBSTACLE CHALLENGE HELPERS
OBSTACLE_MIN_AREA_PX = 1000  # ignore contours smaller than this many pixels - too far away to act on yet
OBSTACLE_TURN = 500

EARLY_LINE_Y1 = 250
EARLY_LINE_Y2 = 380   # sits above bottom_frame's own y-range (380-480), so the lap line is seen earlier/farther away
EARLY_LINE_MIN_PX = 300  # tune on field, mirrors the 500 threshold already used to first decide CW/CCW

#STATES
STRAIGHT = 0
TURNING = 1
LEAVE = 2 
PARK = 3
WAIT = 4   # TURNING saw the block matching this direction's special case; drive straight until the center frame fills with black
FORCE = 5  # force a hard turn for a fixed duration, then hand back to the normal algorithm
TURN_STATES = (TURNING, WAIT, FORCE)  # states where wall detection should be skipped, same as TURNING

state = STRAIGHT
indicator = "STRA"

#wall_frame values
#obstacle frame values  0, 100, 640, 300
wfx1 = 0
wfy1 = 220
wfx2 = 640
wfy2 = 260

# Wall detection dead zone: x-values between these two are NOT scanned for wall pixels.
# Left detection zone = wfx1..wf_gap_left_x, right detection zone = wf_gap_right_x..wfx2.
# Tune these by hand against the camera feed; set both equal for no gap.
wf_gap_left_x = 220
wf_gap_right_x = 420

#bottom-right corner of red block's bounding box (robot passes on its right) or
#bottom-left corner of green block's bounding box (robot passes on its left) - the box's
#own corner is just the block's global min/max x/y, so it doesn't jitter frame-to-frame
#the way a banded scan over raw contour pixels does.
def obstacle_bbox_point(contour, mode):
    x, y, w, h = cv2.boundingRect(contour)
    corner = (x + w, y + h) if mode == 'RED' else (x, y + h)
    return corner, (x, y, w, h)

#right-align debug text against the image's right edge (used for the obstacle status block)
def put_text_right_aligned(image, text, y, right_margin=10, font=cv2.FONT_HERSHEY_SIMPLEX, scale=0.6, color=(0,255,255), thickness=2):
    (text_w, _), _ = cv2.getTextSize(text, font, scale, thickness)
    x = image.shape[1] - text_w - right_margin
    cv2.putText(image, text, (x, y), font, scale, color, thickness)

#find the wall's inner edge at the obstacle's own row, widening the search band if a thin
#slice doesn't turn up enough wall pixels, instead of silently falling back to a stale value
def find_dynamic_wall_x(wall_frame, y_center, side):
    for band_height in (30, 60, 100):
        x = wall_frame.getInnerEdgeAtRow(color=[0,1], y_center=y_center, side=side, band_height=band_height)
        if x is not None:
            return x
    return None

#closest obstacle = whichever red/green contour reaches furthest down the image
def find_closest_obstacle(red_contours, green_contours):
    best = None  # (max_y, color_name, contour)
    for c in red_contours:
        y = int(c[:, :, 1].max())
        if best is None or y > best[0]:
            best = (y, 'RED', c)
    for c in green_contours:
        y = int(c[:, :, 1].max())
        if best is None or y > best[0]:
            best = (y, 'GREEN', c)
    return (best[1], best[2]) if best is not None else None

##### Logic Execution Start

#getting serial connection working
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
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

    #create frames
    # Bottom scan for walls (black + magenta parking block, treated as one wall surface)
    #x1 = 20
    wall_frame = Frame(wfx1, wfy1, wfx2, wfy2, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(255, 0, 0), leftZoneX=wf_gap_left_x, rightZoneX=wf_gap_right_x, source=image_source)
    # keep existing bottom color checks for lap/dir detection
    bottom_frame = Frame(270, 380, 370, 480, image, [lowBlue, lowOrange], [highBlue,  highOrange], source=image_source)
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0)) #blue
    orangePx = bottom_frame.getContour(1, contourColor =(0, 128, 255)) #orange

    #if (lines >= 12):    
    # parking_detection_frame = Frame(0, 240, 320, 480, image, [lowMagenta], [highMagenta])
    # magentaPx = parking_detection_frame.getContour(0, contourColor=(255, 0, 0)) #magenta
    #44800 px 

    # #leaving parking frames (is it CW or CCW?)
    # CW_frame = Frame(40, 300, 120, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0, 255, 0))
    # CCW_frame = Frame(520, 300, 600, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0, 255, 0))
 
    # #CW pixels
    # CWBPx = CW_frame.getContour(0, contourColor=(0, 200, 0)) #black
    # CWPPx = CW_frame.getContour(1, contourColor=(0, 200, 0)) #purple
    # LCWPx = CWBPx + CWPPx

    # #CCW pixels
    # CCWBPx = CCW_frame.getContour(0, contourColor=(0, 200, 0)) #black
    # CCWPPx = CCW_frame.getContour(1, contourColor=(0, 200, 0)) #purple
    # RCCWPx = CCWBPx + CCWPPx

                       
    # # Straight wall detection (black) frame for turning
    # front_frame = Frame(250, 200, 390, 270, image, [lowBlack], [highBlack], frameColor = (255,0,0))
    # frontBlack = front_frame.getContour(0, contourColor=(0,255,0))

    # Use split horizontal scanline to find inner left/right wall edges (black or magenta)
    # Skip wall detection entirely while turning so it can't influence steering/state decisions
    if state not in TURN_STATES:
        left_x, right_x, wall_mask, scan_y1, scan_y2 = wall_frame.getInnerEdgesSplit(color=[0,1], col_threshold=20, contourColor=(255,255,255))
    else:
        left_x, right_x, wall_mask = None, None, None
        scan_y1, scan_y2 = wall_frame.y1, wall_frame.y2

    #-------------------------------------------------------------------------------------
    # OBSTACLE DETECTION (Obstacle Challenge extension) - larger ROI than wall_frame so
    # red/green obstacles are picked up earlier. Runs every frame so debug overlays and
    # side_wall_missing (computed below from the raw wall left_x/right_x) are unaffected.
    # flat color list: 0=red(low hue), 1=green, 2=magenta, 3=red(high hue wraparound)
    obstacle_frame = Frame(0, 100, 640, 380, image, [lowRed1, lowGreen, lowMagenta, lowRed2], [highRed1, highGreen, highMagenta, highRed2], frameColor=(0,255,255), source=image_source)
    #cv2.rectangle(image, (obstacle_frame.x1, obstacle_frame.y1), (obstacle_frame.x2, obstacle_frame.y2), (0,255,255), 2)

    # count raw red/green pixels in the obstacle ROI
    greenPx = obstacle_frame.getContour(1, contourColor=(0,255,0))

    # debug: outline wall_frame's own ROI and draw its raw black contour - must come AFTER
    # obstacle_frame.getContour above, since getContour blits a blurred copy of image_source
    # over its entire ROI (0,100)-(640,380), which fully contains wall_frame's ROI and would
    # erase anything drawn here if drawn earlier in the loop
    cv2.rectangle(image, (wall_frame.x1, wall_frame.y1), (wall_frame.x2, wall_frame.y2), (255, 255, 0), 1)
    wall_contours = wall_frame.getColorContoursSplit(0, min_area=200)
    cv2.drawContours(image, wall_contours, -1, (255, 255, 0), 2)

    hsv_obs_roi = cv2.cvtColor(image_source[obstacle_frame.y1:obstacle_frame.y2, obstacle_frame.x1:obstacle_frame.x2], cv2.COLOR_BGR2HSV)
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv_obs_roi, lowRed1, highRed1),
        cv2.inRange(hsv_obs_roi, lowRed2, highRed2)
    )
    redPx = cv2.countNonZero(red_mask)

    # min_area gates by actual contour pixel count - a block must be at least
    # OBSTACLE_MIN_AREA_PX pixels to be detected at all (too far away otherwise).
    red_contours = obstacle_frame.getColorContours([0,3], min_area=OBSTACLE_MIN_AREA_PX)  # combined red hue ranges
    green_contours = obstacle_frame.getColorContours(1, min_area=OBSTACLE_MIN_AREA_PX)
    magenta_contours = obstacle_frame.getColorContours(2, min_area=200)  # debug only, not used for steering


    # draw both the raw contour (thin, so you can see what the camera actually picked up) and
    # its bounding box (thick) for every detected block - only the box feeds the boundary-point
    # calculation below, the contour outline is purely visual
    cv2.drawContours(image, red_contours, -1, (0,0,255), 1)
    cv2.drawContours(image, green_contours, -1, (0,255,0), 1)
    for c in red_contours:
        bx, by, bw, bh = cv2.boundingRect(c)
        cv2.rectangle(image, (bx, by), (bx + bw, by + bh), (0,0,255), 2)
    for c in green_contours:
        bx, by, bw, bh = cv2.boundingRect(c)
        cv2.rectangle(image, (bx, by), (bx + bw, by + bh), (0,255,0), 2)
    cv2.drawContours(image, magenta_contours, -1, (255,0,255), 2)

    closest_obstacle = find_closest_obstacle(red_contours, green_contours)
    obstacle_mode = 'NORMAL'
    obstacle_point = None
    if closest_obstacle is not None:
        obstacle_mode, obstacle_contour = closest_obstacle
        # obstacle_point is already in full-image coordinates (getColorContours shifts contours
        # by the frame's own x1/y1), matching left_x/right_x from getInnerEdgesSplit.
        obstacle_point, (bx, by, bw, bh) = obstacle_bbox_point(obstacle_contour, obstacle_mode)
        highlight_color = (0,0,255) if obstacle_mode == 'RED' else (0,255,0)  # match the block's own color
        cv2.rectangle(image, (bx, by), (bx + bw, by + bh), highlight_color, 3)  # highlight selected obstacle's box
        # (the obstacle's exact boundary point is drawn once, as part of the corridor line below,
        # to avoid a second redundant marker at a possibly-inconsistent position)
    #-------------------------------------------------------------------------------------

    #count orange line when detected (laps)
    if (orangePx > 200) and (orangePx < 10000):
        if time.time() - start_time_line > 1.5: #if orange line is detected for more than 1 second, count it
            orangeLine += 1
            start_time_line = time.time() #reset timer when orange line is detected
            line_detected = True
    
    #count BLUE line when detected (laps)
    if (bluePx > 200) and (bluePx < 10000):
        if time.time() - start_time_line > 1.5: #if blue line is detected for more than 1 second, count it
            blueLine += 1
            start_time_line = time.time() #reset timer when blue line is detected
            line_detected = True
    
    lines = max(orangeLine, blueLine)

    #CW OR CCW?
    if (CW == 0) and (CCW == 0):
        if 500 < orangePx < 10000:
            CW = 1
        elif 500 < bluePx < 10000:
            CCW = 1
    elif state not in TURN_STATES and (CW == 1) and (CCW == 0):
        side_wall_missing = right_x is None
    elif state not in TURN_STATES and (CW == 0) and (CCW == 1):
        side_wall_missing = left_x is None

    # if (lines >= 12) and (magentaPx >= 5000) and (state != PARK):
    #     state = PARK
    #     saw_turn = time.time()

    #to see if we will turn soon (black frame, if it saw enough of the wall)
    # if (frontBlack > 2000):
    #     frontBlack_detected = True
    if state == STRAIGHT:
        #------------------------------------------------------------------------------------------------------
        # Obstacle avoidance: swap one corridor boundary for the closest obstacle's edge, and
        # re-measure the PAIRED wall at the obstacle's own row instead of the static wall scan
        # band, so the corridor reflects the actual gap at that distance - as the obstacle gets
        # closer (its row moves down-frame), both the obstacle x and the paired wall x are
        # re-read at that same row every frame. Falls back to the static wall_frame reading if
        # the wall genuinely can't be found at that row.
        
        #since values change during getting out of parking
        speed_value = 170
        on = 1
         
        dynamic_row_y = None
        if obstacle_mode in ('RED', 'GREEN') and obstacle_point is not None:
            # Smooth the obstacle's row (not its x) so the corridor line's height doesn't
            # flicker frame to frame with contour-detection noise; ox stays raw since the
            # band-based bottom_right_point/bottom_left_point selection already stabilizes it.
            ox, oy_raw = obstacle_point

            #if it is lager han the min obstacle area size
            if (redPx > OBSTACLE_MIN_AREA_PX) or (greenPx > OBSTACLE_MIN_AREA_PX):
                #wall frame if there is an obstacle (same as obstacle frame) 
                #obstacle frame values  0, 100, 640, 300
                wfx1 = 0
                wfy1 = 150
                wfx2 = 640
                wfy2 = 380

            if filtered_obstacle_y is None:
                filtered_obstacle_y = oy_raw
            filtered_obstacle_y = (obstacle_y_alpha * oy_raw) + ((1 - obstacle_y_alpha) * filtered_obstacle_y)
            oy = int(filtered_obstacle_y)

            if obstacle_mode == 'RED':
                left_x = ox
                dynamic_right = find_dynamic_wall_x(wall_frame, oy, 'right')
                if dynamic_right is not None:
                    right_x = dynamic_right
                    dynamic_row_y = oy
            else:  # GREEN
                right_x = ox
                dynamic_left = find_dynamic_wall_x(wall_frame, oy, 'left')
                if dynamic_left is not None:
                    left_x = dynamic_left
                    dynamic_row_y = oy
        else:
            filtered_obstacle_y = None

            #wall_frame if there are no obstacles
            wfx1 = 0
            wfy1 = 220
            wfx2 = 640
            wfy2 = 260

        # Steering based on corridor center from inner edges with partial-visibility handling
        # Target/error are based on wall_frame's own x-bounds, not the full camera frame
        img_center = (wall_frame.x1 + wall_frame.x2) // 2
        corridor_center = None
        steering_error = 0

        # Determine corridor center depending on which inner edges are available
        if (left_x is not None) and (right_x is not None):
            track_width = abs(right_x - left_x)
            corridor_center = (left_x + right_x) // 2
            last_corridor_center = corridor_center
            last_detection_time = time.time()
            detection_mode = 'both'

        elif (left_x is not None) and (right_x is None):
            # Only left wall visible: project the missing right wall and compute midpoint
            inferred_right = wall_frame.x2
            corridor_center = (left_x + inferred_right) // 2
            # draw inferred right wall for debugging
            cv2.line(image, (int(inferred_right), scan_y1), (int(inferred_right), scan_y2), (0,180,0), 1)
            last_corridor_center = corridor_center
            last_detection_time = time.time()
            detection_mode = 'left_only'

        elif (right_x is not None) and (left_x is None):
            # Only right wall visible: project the missing left wall and compute midpoint
            inferred_left = wall_frame.x1
            corridor_center = (right_x + inferred_left) // 2
            # draw inferred left wall for debugging
            cv2.line(image, (int(inferred_left), scan_y1), (int(inferred_left), scan_y2), (0,180,0), 1)
            last_corridor_center = corridor_center
            last_detection_time = time.time()
            detection_mode = 'right_only'

        else:
            # Neither wall visible: if there's a block, infer the missing wall from the frame edge
            if obstacle_point is not None and obstacle_mode == 'RED':
                left_x = obstacle_point[0]
                inferred_right = wall_frame.x2
                corridor_center = (left_x + inferred_right) // 2
                cv2.line(image, (int(inferred_right), scan_y1), (int(inferred_right), scan_y2), (0,180,0), 1)
                detection_mode = 'red_block_only'
            elif obstacle_point is not None and obstacle_mode == 'GREEN':
                right_x = obstacle_point[0]
                inferred_left = wall_frame.x1
                corridor_center = (right_x + inferred_left) // 2
                cv2.line(image, (int(inferred_left), scan_y1), (int(inferred_left), scan_y2), (0,180,0), 1)
                detection_mode = 'green_block_only'
            else:
                # Neither wall nor block is visible; fall back to image center.
                corridor_center = img_center
                detection_mode = 'none'

        
        # Compute steering error from corridor center
        
        if filtered_center is None:
            filtered_center = corridor_center

        #update filter eery frame
        filtered_center = (alpha * corridor_center) + ((1 - alpha) * filtered_center)
        #use filtered value instead of raw for smoother steering corrections
        steering_error = filtered_center - img_center

        derivative = steering_error - previous_error

        # Dead zone (pixels) around image center where steering is suppressed
        dead_zone_px = 20  # configurable

        # Draw debug overlays. Everything - wall/obstacle boundary markers, the corridor line,
        # the midpoint, and the error line - is drawn on this single row (mid_y) so the picture
        # is a straight horizontal line whose x-values exactly match what corridor_center/
        # steering_error are computed from. No diagonal ever enters the calculation or the draw.
        # mid_y itself moves with the obstacle's own row (dynamic_row_y) when one is active and
        # its paired wall was found there, instead of staying pinned to the static scan band.
        mid_y = dynamic_row_y if dynamic_row_y is not None else (scan_y1 + scan_y2) // 2
        if left_x is not None:
            cv2.circle(image, (left_x, mid_y), 6, (0,255,255), -1)   # active left boundary
        if right_x is not None:
            cv2.circle(image, (right_x, mid_y), 6, (255,0,255), -1)  # active right boundary

        # Corridor line: horizontal line directly between the two active boundaries.
        if (left_x is not None) and (right_x is not None):
            cv2.line(image, (left_x, mid_y), (right_x, mid_y), (0,165,255), 2)

        # corridor_center already IS (left_x + right_x) // 2, so the midpoint marker sits
        # exactly halfway along the corridor line by construction.
        cv2.circle(image, (int(corridor_center), mid_y), 6, (0,255,0), -1)

        cv2.line(image, (img_center, scan_y1), (img_center, scan_y2), (255,255,255), 1) # image center reference line

        # Dead-zone rectangle centered at image center
        dz_x1 = img_center - dead_zone_px
        dz_x2 = img_center + dead_zone_px
        dz_y1 = scan_y1 - 10
        dz_y2 = scan_y2 + 10
        cv2.rectangle(image, (dz_x1, dz_y1), (dz_x2, dz_y2), (200,200,200), 1)

        # If inside dead zone, zero steering error
        if abs(steering_error) <= dead_zone_px:
            steering_error = 0

        # Steering error line: true image center -> calculated midpoint, and nothing else -
        # exactly the existing Open Challenge visualization, on the same wall scan row (mid_y)
        # as every other corridor marker above.
        cv2.line(image, (img_center, mid_y), (int(corridor_center), mid_y), (0,0,255), 2)

        #see the lap line earlier (higher up) than bottom_frame, so there's time to check for the
        #"wrong side" block before actually needing to turn - only start looking for the block
        #once the line matching this direction has been spotted
        early_line_frame = Frame(270, EARLY_LINE_Y1, 370, EARLY_LINE_Y2, image, [lowBlue, lowOrange], [highBlue, highOrange], frameColor=(0,200,200), source=image_source)
        early_bluePx = early_line_frame.getContour(0, contourColor=(255,85,0))
        early_orangePx = early_line_frame.getContour(1, contourColor=(0,128,255))

        #check if it is turning
        if CW and (early_orangePx > EARLY_LINE_MIN_PX) and (greenPx > OBSTACLE_TURN):
            state = WAIT
            # print("************ TURN BLOCK WAIT (early GREEN)")
        elif CCW and (early_bluePx > EARLY_LINE_MIN_PX) and (redPx > OBSTACLE_TURN):
            state = WAIT
            # print("TURN BLOCK WAIT (early RED)")
        elif line_detected:
            if turn_delay_time is None:
                # print("Here")
                turn_delay_time = time.time()

            if time.time() - turn_delay_time > turn_delay:
                state = TURNING
                turning_start_time = time.time()
                turn_delay_time = None  # reset for next detection
                # print("ENTER TURN")
        else:
           
           # Reset if the condition is no longer true
            turn_delay_time = None

            # Stronger PD controller with a capped correction range
            control_signal = (kp * steering_error) + (kd * derivative)
            control_signal = max(-max_steering_correction, min(max_steering_correction, control_signal))
            steering_value = center + control_signal
            previous_error = steering_error  # update for next loop

            steering_error = round(steering_error)
            # print(f"Mode: {detection_mode}, Corridor center: {corridor_center}, Img center: {img_center}, Error: {steering_error}, Steering Value: {steering_value}")
            steering_value = round(steering_value / 5) * 5 #round to nearest 5 for smoother steering
            # print(f"Rounded Steering Value: {steering_value}")

    #------------------------------------------------------------------------------------------------------
    #if the state is turning
    elif state == TURNING:
        if CW:
            steering_value = center + steering_margin
        elif CCW:
            steering_value = center - steering_margin

        if (redPx > OBSTACLE_TURN) or (greenPx > OBSTACLE_TURN): # if it sees blocks while turning
            state = STRAIGHT
            line_detected = False
            # frontBlack_detected = False
            side_wall_missing = False
            previous_error = 0
            filtered_center = None
            filtered_obstacle_y = None
            turn_delay_time = None
            # print("EXIT TURN")
        elif time.time() - turning_start_time > turn_execution_time: #if it has been turning for more than the execution time, go back to straight mode
            state = STRAIGHT
            line_detected = False
            # frontBlack_detected = False
            side_wall_missing = False
            previous_error = 0
            filtered_center = None
            filtered_obstacle_y = None
            turn_delay_time = None
            # print("EXIT TURN")

    #------------------------------------------------------------------------------------------------------
    #saw the "wrong side" block early (during STRAIGHT): go straight until the center frame fills with black
    elif state == WAIT:
        if CW: 
            steering_value = center-5  # keep going straight
        if CCW:
            steering_value = center+5  # keep going straight

        print("################# bro")
        center_frame = Frame(315, 235, 325, 245, image, [lowBlack], [highBlack], frameColor=(0,255,255), source=image_source)
        centerBlackPx = center_frame.getContour(0, contourColor=(0,255,0))
        print("^ ^ ^ ^ ^ ^ ^ ^ ^  Black Pixels - ", centerBlackPx)

        if centerBlackPx >= CENTER_BLACK_FULL_PX:
            state = FORCE
            block_force_turn_start = time.time()
            print("TURN BLOCK FORCE")

    #------------------------------------------------------------------------------------------------------
    #force a hard turn for a fixed duration, then hand back to the normal algorithm
    elif state == FORCE:
        if CW:
            steering_value = center + steering_margin
        elif CCW:
            steering_value = center - steering_margin

        if time.time() - block_force_turn_start > block_force_turn_time:
            state = STRAIGHT
            line_detected = False
            side_wall_missing = False
            previous_error = 0
            filtered_center = None
            filtered_obstacle_y = None
            turn_delay_time = None
            # print("EXIT TURN BLOCK")

    #if the state is leaving parking (starts off in parking)
    elif state == LEAVE:
        speed_value = 0

        #leaving parking frames (is it CW or CCW?)
        CW_frame = Frame(40, 300, 120, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0, 255, 0), source=image_source)
        CCW_frame = Frame(520, 300, 600, 380, image, [lowBlack, lowMagenta], [highBlack, highMagenta], frameColor=(0, 255, 0), source=image_source)
    
        #CW pixels
        CWBPx = CW_frame.getContour(0, contourColor=(0, 200, 0)) #black
        CWPPx = CW_frame.getContour(1, contourColor=(0, 200, 0)) #purple
        LCWPx = CWBPx + CWPPx
    
        #CCW pixels
        CCWBPx = CCW_frame.getContour(0, contourColor=(0, 200, 0)) #black
        CCWPPx = CCW_frame.getContour(1, contourColor=(0, 200, 0)) #purple
        RCCWPx = CCWBPx + CCWPPx

        #CCW or CW (only decide once, so it doesn't flip mid-maneuver)
        if (CW == 0) and (CCW == 0):
            if LCWPx > RCCWPx:
                CW = 1
            else:
                CCW = 1

        if CW:
            #start the leave timer once, the first time we enter this state
            if leave_start_time is None:
                leave_start_time = time.time()

            elapsed = time.time() - leave_start_time

            if elapsed < leave_forward_time:
                #phase 1: drive straight out for leave_forward_time seconds
                speed_value = 135
                steering_value = center + steering_margin 
                on = 1
            # elif elapsed < leave_forward_time + leave_turn_time:
            #     #phase 2: turn for leave_turn_time seconds
            #     speed_value = 160
            #     steering_value = 120
            #     on = 1
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2:
            #     #phase 3: turn inward other way
            #     speed_value = 160
            #     steering_value = 40
            #     on = 2
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2 + leave_turn_time3:
            #     #phase 4: turn out leave
            #     speed_value = 160
            #     steering_value = 120
            #     on = 1
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2 + leave_turn_time3 + leave_turn_time4:
            #     #phase 5: leave, enter field
            #     speed_value = 160
            #     steering_value = 40
            #     on = 1
            else:
                #done leaving, hand off to normal straight driving
                state = STRAIGHT
                leave_stat_rtime = None
        else: #if CCW
            #start the leave timer once, the first time we enter this state
            if leave_start_time is None:
                leave_start_time = time.time()

            elapsed = time.time() - leave_start_time

            if elapsed < leave_forward_time:
                #phase 1: drive straight out for leave_forward_time seconds
                speed_value = 135
                steering_value = center - steering_margin
                on = 1
            # elif elapsed < leave_forward_time + leave_turn_time:
            #     #phase 2: turn for leave_turn_time seconds
            #     speed_value = 160
            #     steering_value = 40
            #     on = 1
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2:
            #     #phase 3: turn inward other way
            #     speed_value = 160
            #     steering_value = 120
            #     on = 2
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2 + leave_turn_time3:
            #     #phase 4: turn out leave
            #     speed_value = 160
            #     steering_value = 40
            #     on = 1
            # elif elapsed < leave_forward_time + leave_turn_time + leave_turn_time2 + leave_turn_time3 + leave_turn_time4:
            #     #phase 5: leave, enter field
            #     speed_value = 160
            #     steering_value = 120
            #     on = 1
            else:
                #done leaving, hand off to normal straight driving
                state = STRAIGHT
                leave_start_time = None
    elif state == PARK:
        if (saw_turn < 2.1):
            speed_value = 160
            steering_value = center + steering_margin
        


    # #stop when see all the orange line
    if lines >=  12:
        if start_time_finshed == 0:
            start_time_finshed = time.time() #start timer when all orange lines are detected
        if time.time() - start_time_finshed > 3: #if all orange lines are detected for more than 3 seconds, stop the car
            speed_value = 0 


    # Hello Shadyta, this is your coach suffering with the servo
    #Straight angle is 85 dont question why :)
    #max andgles will be 135 and 55
    if steering_value > (center + steering_margin):
        steering_value = (center + steering_margin)

    elif steering_value < (center - steering_margin):
        steering_value = (center - steering_margin)

    #print values on camera feed
    cv2.putText(image, f"Lap: {math.ceil((lines/4))}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Direction: {'CW' if CW else 'CCW'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"State: {state}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    #obstacle challenge debug text (top right)
    put_text_right_aligned(image, f"Obstacle Mode: {obstacle_mode}", 30)
    put_text_right_aligned(image, f"Midpoint X: {corridor_center}", 55)
    put_text_right_aligned(image, f"Steer Error: {steering_error}", 80)

    #send values to micrbit through serial connections
    # print(f"Steering: {steering_value}, Speed: {speed_value}, ON?: {on}, LINES: {lines}, STATE: {state}")
    # print(f"Line Detected: {line_detected}, Empty Wall: {side_wall_missing}, cw: {CW}, ccw: {CCW}\n")

    indicator = state

    message = (f"{steering_value} {speed_value} {on} {lines} {indicator}\n")

    # only send if different
    if message != last_message:
        ser.write(message.encode())
        ser.flush()

        last_message = message

    time.sleep(0.05)

    if DRAW:
        #display camera feed after processing
        cv2.imshow("Camera Feed", image)

    # Check for 'q' key press to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        ser.write(f"85 0 1 {lines} OPEN\n".encode())
        time.sleep(0.05)
        break


ser.write(f"85 0 1 {lines} OPEN\n".encode())
print("FINISH")
time.sleep(0.10)
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done
