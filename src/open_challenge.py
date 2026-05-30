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

#COUNTING
CW = 0
CCW = 0

orangeLine = 0
blueLine = 0


#COLOR VALUES
lowBlack  = np.array([0, 0, 0])
highBlack = np.array([180, 255, 70])

lowBlue  = np.array([100, 70, 50])
highBlue = np.array([130, 255, 255])

lowOrange  = np.array([8, 100, 100])
highOrange = np.array([35, 255, 255])

#MICROBIT VALUES
steering_value = 0
speed_value = 0
on = 1

#message sent to serial
last_message = ""

#wall error kp steering
#og is 0.015
kp = 0.015
center = 90 #center value for steering, adjust as needed

#timer variables
start_time_line = time.time()
start_time_finshed = 0

#main loop to show camera feed
while True:

    #camera feed
    image = picam2.capture_array()
    image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    # flip upside down
    image = cv2.rotate(image, cv2.ROTATE_180)
    

    #create frame and get contours
    frame1 = Frame(0, 170, 40, 480,image, [lowBlack], [highBlack], frameColor=(255, 0, 0))
    leftPx = frame1.getContour()

    frame2 = Frame(600, 170, 640, 480,image, [lowBlack], [highBlack], frameColor=(255, 0, 0))
    rightPx = frame2.getContour()

    bottom_frame = Frame(120, 370, 520, 470, image, [lowBlue, lowOrange], [highBlue,  highOrange])
    bluePx = bottom_frame.getContour(0, contourColor=(255, 85, 0)) #blue
    orangePx = bottom_frame.getContour(1, contourColor =(0, 128, 255)) #orange


    #change later
    if (orangePx > 10000) and (orangePx < 50000):
        if time.time() - start_time_line > 1.5: #if orange line is detected for more than 1 second, count it
            orangeLine += 1
            start_time_line = time.time() #reset timer when orange line is detected
    
    
    # #CW OR CCW?
    # if (CW == 0) and (CCW == 0):)
    #     if (orangePx < 10000) and (orangePx > 500):
    #         CW = 1
            
    #     elif (bluePx < 10000) and (bluePx > 500):
    #         CCW = 1

    #steering value based off of wall error
    error = leftPx - rightPx

    steering_value = center + (kp * error)

    steering_value = round(steering_value / 10) * 10 #round to nearest 10 for smoother steering


    #stop when see all the orange line
    if orangeLine >=  4:
        if start_time_finshed == 0:
            start_time_finshed = time.time() #start timer when all orange lines are detected
        if time.time() - start_time_finshed > 8.2: #if all orange lines are detected for more than 3 seconds, stop the car
            speed_value = 100 
            on = 0

    #print values on camera feed
    cv2.putText(image, f"Lap: {orangeLine}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(image, f"Steering: {steering_value}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


    #send values to micrbit through serial connections
    print(f"Steering: {steering_value}, Speed: {speed_value}, ON?: {on}")
    print(orangeLine)

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
        time.sleep(0.05)
        break


steering_value1 = 90
speed_value1 = 100
on1 = 0
ser.write(f"SERVO:{steering_value1},SPEED:{speed_value1},{on1}\n".encode())
time.sleep(0.05)
ser.close() #close serial connection when done

cv2.destroyAllWindows() #clean up windows when done
