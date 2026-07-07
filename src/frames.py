import time
import numpy as np
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

    # Scan a horizontal band near the bottom of this frame to find left/right wall edges
    def getHorizontalEdges(self, color=0, scan_height=30, col_threshold=20, contourColor=(0, 255, 0)):
        """
        Scans a horizontal band at the bottom of the frame ROI and returns left and right x
        coordinates (in image coordinates) where the specified color appears.

        Returns (left_x, right_x, mask, scan_y1, scan_y2) where left_x/right_x may be None
        if no edges are detected.
        """
        # Determine scanning region (last `scan_height` pixels of this frame)
        scan_y1 = max(self.y1, self.y2 - scan_height)
        scan_y2 = self.y2

        # ROI for the horizontal scan
        roi = self.image[scan_y1:scan_y2, self.x1:self.x2]
        if roi.size == 0:
            return None, None, None, scan_y1, scan_y2

        # Smooth and convert to HSV
        blurred = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Build mask for the requested color index
        mask = cv2.inRange(hsv, self.lowColor[color], self.highColor[color])

        # Clean small noise
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Sum vertically to get column activity (higher means more color pixels in that column)
        col_sum = np.sum(mask, axis=0)
        cols = np.where(col_sum > col_threshold)[0]

        # Draw the scanning band rectangle for debugging
        cv2.rectangle(self.image, (self.x1, scan_y1), (self.x2, scan_y2), self.frameColor, 1)

        # No edges found
        if cols.size == 0:
            return None, None, mask, scan_y1, scan_y2

        left_col = int(cols[0])
        right_col = int(cols[-1])

        # Convert back to image coordinates
        left_x = self.x1 + left_col
        right_x = self.x1 + right_col

        # Visual overlays for the scan band and edge lines
        cv2.line(self.image, (left_x, scan_y1), (left_x, scan_y2), contourColor, 2)
        cv2.line(self.image, (right_x, scan_y1), (right_x, scan_y2), contourColor, 2)

        return left_x, right_x, mask, scan_y1, scan_y2

    def getInnerEdgesSplit(self, color=0, scan_height=30, col_threshold=20, contourColor=(0, 255, 0), min_contour_area=150):
        """
        Split the bottom scanning band into a left ROI and a right ROI, find the largest
        contour in each ROI for the requested color, and return the inner edges:
        - left inner edge: the rightmost x of the largest left contour (closest to center)
        - right inner edge: the leftmost x of the largest right contour (closest to center)

        Returns (left_x, right_x, full_mask, scan_y1, scan_y2) where left_x/right_x may be None.
        """
        # Determine scanning region (last `scan_height` pixels of this frame)
        scan_y1 = max(self.y1, self.y2 - scan_height)
        scan_y2 = self.y2

        # ROI widths and mid column
        total_width = self.x2 - self.x1
        if total_width <= 0 or scan_y2 <= scan_y1:
            return None, None, None, scan_y1, scan_y2

        mid_col = self.x1 + total_width // 2

        # Left and right ROI slices
        left_roi = self.image[scan_y1:scan_y2, self.x1:mid_col]
        right_roi = self.image[scan_y1:scan_y2, mid_col:self.x2]

        # Helper to process a roi and return inner edge and its mask
        def _process_roi(roi, find_rightmost, area_thresh):
            if roi.size == 0:
                return None, None
            blurred = cv2.GaussianBlur(roi, (7, 7), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lowColor[color], self.highColor[color])
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                return None, mask
            # choose the largest contour by area
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) < area_thresh:
                return None, mask
            # compute inner edge in local ROI coordinates
            xs = largest[:, :, 0].reshape(-1)
            edge_local = int(xs.max()) if find_rightmost else int(xs.min())
            return edge_local, mask

        left_edge_local, left_mask = _process_roi(left_roi, find_rightmost=True, area_thresh=min_contour_area)
        right_edge_local, right_mask = _process_roi(right_roi, find_rightmost=False, area_thresh=min_contour_area)

        # Build a combined mask that spans the full frame ROI width
        full_mask = np.zeros((scan_y2 - scan_y1, total_width), dtype=np.uint8)
        if left_mask is not None:
            full_mask[:, :mid_col - self.x1] = left_mask
        if right_mask is not None:
            full_mask[:, mid_col - self.x1:] = right_mask

        # Visual overlays: draw the left and right ROI rectangles
        cv2.rectangle(self.image, (self.x1, scan_y1), (mid_col, scan_y2), (255, 200, 0), 1)
        cv2.rectangle(self.image, (mid_col, scan_y1), (self.x2, scan_y2), (0, 200, 255), 1)

        left_x = None
        right_x = None

        # If left contour found, draw it (shift not required when drawing into left ROI slice)
        if left_edge_local is not None:
            left_x = self.x1 + left_edge_local
            # draw vertical inner-edge line for left
            cv2.line(self.image, (left_x, scan_y1), (left_x, scan_y2), contourColor, 2)
            # draw contours for debugging
            contours_l, _ = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_l:
                cv2.drawContours(self.image[scan_y1:scan_y2, self.x1:mid_col], contours_l, -1, contourColor, 2)

        # If right contour found, draw it (offset when drawing into combined ROI)
        if right_edge_local is not None:
            right_x = mid_col + right_edge_local
            cv2.line(self.image, (right_x, scan_y1), (right_x, scan_y2), contourColor, 2)
            contours_r, _ = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_r:
                # draw into combined ROI area (offset columns by mid_col - self.x1)
                draw_area = self.image[scan_y1:scan_y2, self.x1:self.x2]
                shifted = []
                offset = mid_col - self.x1
                for c in contours_r:
                    shifted.append(c + np.array([[[offset, 0]]]))
                cv2.drawContours(draw_area, shifted, -1, contourColor, 2)

        return left_x, right_x, full_mask, scan_y1, scan_y2


    
    def getWallEdgesSplit(self, color=0, scan_height=30, col_threshold=20,
                      contourColor=(0, 255, 0)):

        scan_y1 = max(self.y1, self.y2 - scan_height)
        scan_y2 = self.y2

        total_width = self.x2 - self.x1

        if total_width <= 0:
            return None, None

        mid_col = self.x1 + total_width // 2

        # Left and right boxes
        left_roi = self.image[scan_y1:scan_y2, self.x1:mid_col]
        right_roi = self.image[scan_y1:scan_y2, mid_col:self.x2]

        def create_mask(roi):

            if roi.size == 0:
                return None

            blurred = cv2.GaussianBlur(roi, (7, 7), 0)

            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(
                hsv,
                self.lowColor[color],
                self.highColor[color]
            )

            kernel = np.ones((3, 3), np.uint8)

            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_OPEN,
                kernel
            )

            return mask

        left_mask = create_mask(left_roi)
        right_mask = create_mask(right_roi)

        left_x = None
        right_x = None

        # LEFT BOX
        if left_mask is not None:

            cols = np.where(
                np.sum(left_mask, axis=0) > col_threshold
            )[0]

            if cols.size > 0:

                # rightmost black pixel in left box
                left_x = self.x1 + int(cols.max())

                cv2.line(
                    self.image,
                    (left_x, scan_y1),
                    (left_x, scan_y2),
                    contourColor,
                    2
                )

        # RIGHT BOX
        if right_mask is not None:

            cols = np.where(
                np.sum(right_mask, axis=0) > col_threshold
            )[0]

            if cols.size > 0:

                # leftmost black pixel in right box
                right_x = mid_col + int(cols.min())

                cv2.line(
                    self.image,
                    (right_x, scan_y1),
                    (right_x, scan_y2),
                    contourColor,
                    2
                )

        # Draw debug boxes
        cv2.rectangle(
            self.image,
            (self.x1, scan_y1),
            (mid_col, scan_y2),
            (255, 0, 0),
            2
        )

        cv2.rectangle(
            self.image,
            (mid_col, scan_y1),
            (self.x2, scan_y2),
            (0, 0, 255),
            2
        )

        return left_x, right_x