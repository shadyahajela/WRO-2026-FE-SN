import time
from picamera2 import Picamera2
import cv2

class Frame:
    def __init__(self, x1, y1, x2, y2, image, lowColor, highColor, frameColor = (0, 0, 255)):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.image = image
        self.lowColor = lowColor  # List of low color values or single tuple
        self.highColor = highColor  # List of high color values or single tuple
        self.mask = None
        self.frameColor = frameColor

    # Function to get contours and count pixels of a specific color within the frame
    def getContour(self, color = 0, isRed = False, contourColor=(0, 0, 255)):

        # Extract ROI
        roi = self.image[self.y1:self.y2, self.x1:self.x2]
        # Blur ROI
        blurred_roi = cv2.GaussianBlur(roi, (7, 7), 0)
        # Replace ROI in original image
        self.image[self.y1:self.y2, self.x1:self.x2] = blurred_roi
        
        # Detect contours within the ROI
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        #Preset value for mask, each color
        self.mask = cv2.inRange(hsv_roi, self.lowColor[color], self.highColor[color])
 
        #If is red
        if isRed:
            maskRed1 = cv2.inRange(hsv_roi, self.lowColor[2], self.highColor[2])
            self.mask = cv2.bitwise_or(maskRed1, self.mask)

        # #Handling other color
        # if self.lowColor2 != None:
        #     mask2 = cv2.inRange(hsv_roi, self.lowColor2[0], self.highColor2[0])
        #     contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #     cv2.drawContours(self.image[self.y1:self.y2, self.x1:self.x2], contours2, -1, contourColor2, 2)
        
        contours, _ = cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Draw contours on the ROI
        cv2.drawContours(self.image[self.y1:self.y2, self.x1:self.x2], contours, -1, contourColor, 2)

        # Draw frame
        cv2.rectangle(self.image, (self.x1, self.y1), (self.x2, self.y2), self.frameColor, 1)

        #counting pixels
        pixels = self.detectColorPixels()

        return pixels

    #counting pixels functions function defining
    def detectColorPixels(self):

        pixel_count = cv2.countNonZero(self.mask)

        return pixel_count