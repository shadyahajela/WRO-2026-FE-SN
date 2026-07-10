#IMPORTS
import time 

import serial

import numpy as np #where you store hue/color values

from frames import Frame #define your color ranges and other frame-related functions

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
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)

#What direction are we going? CW or CCW?
CW = 0
CCW = 0

orangeLine = 0
blueLine = 0
line_detected = False
frontBlack_detected = False
side_wall_missing = False


#COLOR VALUES
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 70])

lowBlue  = np.array([100, 70, 50])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([8, 100, 100])
highOrange = np.array([35, 255, 255])

#MICROBIT VALUES
steering_value = 0 # Calculated steering value to add or subtract from center value
center = 85 #center value for steering, adjust as needed
speed_value = 40 # Speed perce
on = 2

#Last sent message sent to serial to compare against current message to avoid sending duplicates
last_message = ""

#wall error kp steering
kp = 0.3 #og is 0.015
kd = 0.15
previous_error = 0 

alpha = 0.15 # loss pass filter

# Lane estimation for partial visibility handling
track_width = 240

#estimated_lane_width = 240  # initial guess in pixels (tweakable)
#lane_width_alpha = 0.05      # smoothing factor for running average

last_corridor_center = None
last_detection_time = 0
filtered_center = None

#timer variables
start_time_line = time.time()
start_time_finshed = 0

turning_start_time = 0

#STATES
STRAIGHT = 0
TURNING = 1

state = STRAIGHT

#main loop to show camera feed
while True:

    #camera feed
    image = picam2.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    # flip upside down
    image = cv2.rotate(image, cv2.ROTATE_180)
    

    #create frames
    # Bottom scan for walls (black) across much of the image width
    wall_frame = Frame(40,250, 600, 360, image, [lowBlack], [highBlack], frameColor=(255, 0, 0))
    # keep existing bottom color checks for lap/dir detection
    bottom_frame = Frame(120, 370, 520, 470, image, [lowBlue, lowOrange], [highBlue,  highOrange])
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0)) #blue
    orangePx = bottom_frame.getContour(1, contourColor =(0, 128, 255)) #orange

    # # Straight wall detection (black) frame for turning
    # front_frame = Frame(250, 200, 390, 270, image, [lowBlack], [highBlack], frameColor = (255,0,0))
    # frontBlack = front_frame.getContour(0, contourColor=(0,255,0))

    # Use split horizontal scanline to find inner left/right wall edges (black)
    left_x, right_x, wall_mask, scan_y1, scan_y2 = wall_frame.getInnerEdgesSplit(color=0, scan_height=40, col_threshold=20, contourColor=(0,255,0))


    #count orange line when detected (laps)
    if (orangePx > 4000) and (orangePx < 40000):
        if time.time() - start_time_line > 1.5: #if orange line is detected for more than 1 second, count it
            orangeLine += 1
            start_time_line = time.time() #reset timer when orange line is detected
            line_detected = True
    
    #count BLUE line when detected (laps)
    if (bluePx > 4000) and (bluePx < 40000):
        if time.time() - start_time_line > 1.5: #if blue line is detected for more than 1 second, count it
            blueLine += 1
            start_time_line = time.time() #reset timer when blue line is detected
            line_detected = True
    
    #CW OR CCW?
    if (CW == 0) and (CCW == 0):
        if 500 < orangePx < 10000:
            CW = 1
        elif 500 < bluePx < 10000:
            CCW = 1
    elif (CW == 1) and (CCW == 0):
        side_wall_missing = right_x is None
    elif (CW == 0) and (CCW == 1):
        side_wall_missing = left_x is None

    #to see if we will turn soon (black frame, if it saw enough of the wall)
    # if (frontBlack > 2000):
    #     frontBlack_detected = True

    if state == STRAIGHT:
        #------------------------------------------------------------------------------------------------------
        # Steering based on corridor center from inner edges with partial-visibility handling
        img_center = image.shape[1] // 2
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
            inferred_right = image.shape[1]
            corridor_center = (left_x + inferred_right) // 2
            # draw inferred right wall for debugging
            cv2.line(image, (int(inferred_right), scan_y1), (int(inferred_right), scan_y2), (0,180,0), 1)
            last_corridor_center = corridor_center
            last_detection_time = time.time()
            detection_mode = 'left_only'

        elif (right_x is not None) and (left_x is None):
            # Only right wall visible: project the missing left wall and compute midpoint
            inferred_left = 0
            corridor_center = (right_x + inferred_left) // 2
            # draw inferred left wall for debugging
            cv2.line(image, (int(inferred_left), scan_y1), (int(inferred_left), scan_y2), (0,180,0), 1)
            last_corridor_center = corridor_center
            last_detection_time = time.time()
            detection_mode = 'right_only'

        else:
            # Neither wall visible: fall back to last known corridor center or image center
            corridor_center = img_center
            detection_mode = 'none'

        
        # Compute steering error from corridor center
        
        if filtered_center is None:
            filtered_center = corridor_center

        #update filter eery frame
        filtered_center = (alpha * corridor_center) + ((1 - alpha) * filtered_center)
        #use filtered alue instead of raw
        steering_error =  filtered_center - img_center


        derivative = steering_error - previous_error

        # Dead zone (pixels) around image center where steering is suppressed
        dead_zone_px = 20  # configurable

        # Draw debug overlays depending on detections
        # inner-edge markers
        if left_x is not None:
            cv2.circle(image, (left_x, (scan_y1+scan_y2)//2), 6, (0,255,255), -1)    # left wall inner-edge
        if right_x is not None:
            cv2.circle(image, (right_x, (scan_y1+scan_y2)//2), 6, (255,0,255), -1)   # right wall inner-edge

        # corridor center and image center
        cv2.circle(image, (int(corridor_center), (scan_y1+scan_y2)//2), 6, (0,255,0), -1) # corridor center
        cv2.line(image, (img_center, scan_y1), (img_center, scan_y2), (255,255,255), 1) # image center line

        # Dead-zone rectangle centered at image center
        dz_x1 = img_center - dead_zone_px
        dz_x2 = img_center + dead_zone_px
        dz_y1 = scan_y1 - 10
        dz_y2 = scan_y2 + 10
        cv2.rectangle(image, (dz_x1, dz_y1), (dz_x2, dz_y2), (200,200,200), 1)

        # If inside dead zone, zero steering error
        if abs(steering_error) <= dead_zone_px:
            steering_error = 0

        # Draw error line (image center -> corridor center)
        cv2.line(image, (img_center, (scan_y1+scan_y2)//2), (int(corridor_center), (scan_y1+scan_y2)//2), (0,0,255), 2)

        #check if it is turning
        if line_detected and side_wall_missing:
            
            turn_delay_time = time.time() 
            if time.time() - turn_delay_time > 0.75: #wait for 0.5 seconds before turning
                state = TURNING
                turning_start_time = time.time()
                print("ENTER TURN")
        else:
            # Proportional controller
            steering_value = center + (kp * steering_error) + (kd * derivative)
            previous_error = steering_error  # update for next loop

            steering_error = round(steering_error)
            print(f"Mode: {detection_mode}, Corridor center: {corridor_center}, Img center: {img_center}, Error: {steering_error}, Steering Value: {steering_value}")
            steering_value = round(steering_value / 5) * 5 #round to nearest 5 for smoother steering
            print(f"Rounded Steering Value: {steering_value}")

    #------------------------------------------------------------------------------------------------------
    #if the state is turning
    elif state == TURNING:
        if CW:
            steering_value = 125
        elif CCW:
            steering_value = 55

        if time.time() - turning_start_time > 1.7        : #if it has been turning for more than 3 seconds, go back to straight mode
            state = STRAIGHT
            line_detected = False
            # frontBlack_detected = False
            side_wall_missing = False
            print("EXIT TURN")
    

    #stop when see all the orange line
    if orangeLine >=  3:
        if start_time_finshed == 0:
            start_time_finshed = time.time() #start timer when all orange lines are detected
        if time.time() - start_time_finshed > 8.2: #if all orange lines are detected for more than 3 seconds, stop the car
            speed_value = 0 
            on = 0

    #print values on camera feed
    cv2.putText(image, f"Lap: {orangeLine}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Direction: {'CW' if CW else 'CCW'}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"State: {state}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    #send values to micrbit through serial connections
    print(f"Steering: {steering_value}, Speed: {speed_value}, ON?: {on}")
    print(f"Line Detected: {line_detected}, Black detected: {frontBlack_detected}, Empty Wall: {side_wall_missing}, ccw: {CW}, ccw: {CCW}")


    # Hello Shadyta, this is your coach suffering with the servo
    #Straight angle is 95 dont question why :)
    #max andgles will be 135 and 55 
    if steering_value > 125:
        steering_value = 125

    elif steering_value < 65:
        steering_value = 65

    message = f"SERVO:{steering_value},SPEED:{speed_value},{on}\n"

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
        steering_value1 = 85
        speed_value1 = 0
        on1 = 0
        ser.write(f"SERVO:{steering_value1},SPEED:{speed_value1},{on1}\n".encode())
        time.sleep(0.05)
        break


steering_value1 = 85
speed_value1 = 0
on1 = 0
ser.write(f"SERVO:{steering_value1},SPEED:{speed_value1},{on1}\n".encode())
print("FINISH")
time.sleep(0.10)
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done
