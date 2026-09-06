import time
import numpy as np
from picamera2 import Picamera2
import cv2

class Frame:
    # constructor
    def __init__(self, x1, y1, x2, y2, image, lowColor, highColor, frameColor = (0, 0, 255), leftZoneX=None, rightZoneX=None, source=None):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.image = image
        # pixels read for color thresholding/contour extraction come from `source` (a pristine
        # pre-draw snapshot), while `self.image` stays the live buffer overlays get drawn onto -
        # otherwise an earlier Frame's debug line/rectangle can physically cut through a later
        # Frame's color mask if their regions overlap. Defaults to `image` for old call sites.
        self.source = source if source is not None else image
        self.lowColor = lowColor  # List of low color values or single tuple
        self.highColor = highColor  # List of high color values or single tuple
        self.mask = None
        self.frameColor = frameColor

        # Manual left/right detection-zone split (for getInnerEdgesSplit/getInnerEdgeAtRow):
        # x1..leftZoneX is scanned as the left zone, rightZoneX..x2 as the right zone, and
        # leftZoneX..rightZoneX in between is a dead zone that is never scanned. Defaults to
        # the frame's own midpoint (no gap) when not given, matching the old auto-bisect behavior.
        mid = x1 + (x2 - x1) // 2
        self.leftZoneX = mid if leftZoneX is None else max(x1, min(leftZoneX, x2))
        self.rightZoneX = mid if rightZoneX is None else max(self.leftZoneX, min(rightZoneX, x2))

    # Function to get contours and count pixels of a specific color within the frame
    def getContour(self, color = 0, isRed = False, contourColor=(0, 0, 255)):

        # Extract ROI
        roi = self.source[self.y1:self.y2, self.x1:self.x2]
        # Blur ROI
        blurred_roi = cv2.GaussianBlur(roi, (7, 7), 0)
        # Replace ROI in original image
        #self.image[self.y1:self.y2, self.x1:self.x2] = blurred_roi
        
        # Detect contours within the ROI
        hsv_roi = cv2.cvtColor(blurred_roi, cv2.COLOR_BGR2HSV)
        
        #Preset value for mask, each color
        mask = cv2.inRange(hsv_roi, self.lowColor[color], self.highColor[color])
 
        #If is red
        if isRed:
            maskRed1 = cv2.inRange(hsv_roi, self.lowColor[2], self.highColor[2])
            mask = cv2.bitwise_or(maskRed1, mask)

        # #Handling other color
        # if self.lowColor2 != None:
        #     mask2 = cv2.inRange(hsv_roi, self.lowColor2[0], self.highColor2[0])
        #     contours2, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #     cv2.drawContours(self.image[self.y1:self.y2, self.x1:self.x2], contours2, -1, contourColor2, 2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Draw contours on the ROI
        cv2.drawContours(self.image[self.y1:self.y2, self.x1:self.x2], contours, -1, contourColor, 2)

        # Draw frame
        cv2.rectangle(self.image, (self.x1, self.y1), (self.x2, self.y2), self.frameColor, 1)

        #counting pixels
        pixels = self.detectColorPixels(mask)

        return pixels

    #counting pixels functions function defining
    def detectColorPixels(self, mask):

        pixel_count = cv2.countNonZero(mask)

        return pixel_count


    def getInnerEdgesSplit(self, color=0, scan_height=None, col_threshold=20, contourColor=(0, 255, 0), min_contour_area=150):
        """
        Split the frame into a left ROI and a right ROI, find the largest contour in each
        ROI for the requested color, and return the inner edges:
        - left inner edge: the rightmost x of the largest left contour (closest to center)
        - right inner edge: the leftmost x of the largest right contour (closest to center)

        By default, this scans the full frame height. If scan_height is provided, it scans
        only the bottom scan_height pixels of the frame.

        Returns (left_x, right_x, full_mask, scan_y1, scan_y2) where left_x/right_x may be None.
        """
        if scan_height is None:
            scan_y1 = self.y1
        else:
            scan_y1 = max(self.y1, self.y2 - scan_height)
        scan_y2 = self.y2

        # ROI widths and manual left/right zone split
        total_width = self.x2 - self.x1
        if total_width <= 0 or scan_y2 <= scan_y1:
            return None, None, None, scan_y1, scan_y2

        # Left and right ROI slices, with a dead zone (self.leftZoneX..self.rightZoneX) between
        # them that is never scanned
        left_roi = self.source[scan_y1:scan_y2, self.x1:self.leftZoneX]
        right_roi = self.source[scan_y1:scan_y2, self.rightZoneX:self.x2]

        # Allow `color` to be a single index or a list/tuple of indices to OR together
        # (e.g. black + magenta both counting as "wall")
        color_indices = color if isinstance(color, (list, tuple)) else [color]

        # Helper to process a roi and return inner edge and its mask
        def _process_roi(roi, find_rightmost, area_thresh):
            if roi.size == 0:
                return None, None
            blurred = cv2.GaussianBlur(roi, (7, 7), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = None
            for idx in color_indices:
                m = cv2.inRange(hsv, self.lowColor[idx], self.highColor[idx])
                mask = m if mask is None else cv2.bitwise_or(mask, m)
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

        # Build a combined mask that spans the full frame ROI width (dead zone columns stay zero)
        full_mask = np.zeros((scan_y2 - scan_y1, total_width), dtype=np.uint8)
        if left_mask is not None:
            full_mask[:, :self.leftZoneX - self.x1] = left_mask
        if right_mask is not None:
            full_mask[:, self.rightZoneX - self.x1:] = right_mask

        # Visual overlays: draw the left/right ROI rectangles and the dead zone between them
        cv2.rectangle(self.image, (self.x1, scan_y1), (self.leftZoneX, scan_y2), (255, 200, 0), 1)
        cv2.rectangle(self.image, (self.rightZoneX, scan_y1), (self.x2, scan_y2), (0, 200, 255), 1)
        if self.rightZoneX > self.leftZoneX:
            cv2.rectangle(self.image, (self.leftZoneX, scan_y1), (self.rightZoneX, scan_y2), (120, 120, 120), 1)

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
                cv2.drawContours(self.image[scan_y1:scan_y2, self.x1:self.leftZoneX], contours_l, -1, contourColor, 2)

        # If right contour found, draw it (offset when drawing into combined ROI)
        if right_edge_local is not None:
            right_x = self.rightZoneX + right_edge_local
            cv2.line(self.image, (right_x, scan_y1), (right_x, scan_y2), contourColor, 2)
            contours_r, _ = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_r:
                # draw into combined ROI area (offset columns by rightZoneX - x1)
                draw_area = self.image[scan_y1:scan_y2, self.x1:self.x2]
                shifted = []
                offset = self.rightZoneX - self.x1
                for c in contours_r:
                    shifted.append(c + np.array([[[offset, 0]]]))
                cv2.drawContours(draw_area, shifted, -1, contourColor, 2)

        return left_x, right_x, full_mask, scan_y1, scan_y2


    def getColorContours(self, color, min_area=200):
        """
        Generic contour finder scoped to this frame's ROI for a color index (or a
        list of indices to OR together, e.g. for a hue-wraparound color like red).
        Returns a list of contours (each shifted into full-image coordinates) whose
        area is >= min_area. Does not draw anything - callers own their own debug
        visualization so the same detection can be reused for different purposes.
        """
        roi = self.source[self.y1:self.y2, self.x1:self.x2]
        if roi.size == 0:
            return []

        color_indices = color if isinstance(color, (list, tuple)) else [color]

        blurred = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = None
        for idx in color_indices:
            m = cv2.inRange(hsv, self.lowColor[idx], self.highColor[idx])
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= min_area]

        # Shift into full-image coordinates so results compose with other Frame outputs
        offset = np.array([[[self.x1, self.y1]]])
        return [c + offset for c in contours]

    def getColorContoursSplit(self, color, min_area=200):
        """
        Like getColorContours, but scoped to the left/right zones only (self.x1:self.leftZoneX
        and self.rightZoneX:self.x2), skipping the dead zone between them - the same
        leftZoneX/rightZoneX split getInnerEdgesSplit uses, so tuning those two values changes
        both the inner-edge scan and this contour detection together.
        """
        color_indices = color if isinstance(color, (list, tuple)) else [color]

        def _contours_in(roi, offset_x):
            if roi.size == 0:
                return []
            blurred = cv2.GaussianBlur(roi, (7, 7), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            mask = None
            for idx in color_indices:
                m = cv2.inRange(hsv, self.lowColor[idx], self.highColor[idx])
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) >= min_area]
            offset = np.array([[[offset_x, self.y1]]])
            return [c + offset for c in contours]

        left_roi = self.source[self.y1:self.y2, self.x1:self.leftZoneX]
        right_roi = self.source[self.y1:self.y2, self.rightZoneX:self.x2]
        return _contours_in(left_roi, self.x1) + _contours_in(right_roi, self.rightZoneX)



    def getInnerEdgeAtRow(self, color, y_center, side, band_height=30, min_contour_area=40):
        """
        Like the single-side logic inside getInnerEdgesSplit, but scans a band centered on a
        caller-supplied y (full-image coordinates) instead of this frame's own bottom band.
        Lets a wall edge be re-measured at whatever row an obstacle's boundary point sits on,
        so the corridor width reflects the wall gap at that distance rather than a fixed row.

        side: 'left' or 'right' - which zone of this frame's x1:x2 width to search, same
        leftZoneX/rightZoneX split convention as getInnerEdgesSplit (rightmost pixel of the
        left zone = inner edge, and vice versa; the dead zone between them is never searched).

        Returns the edge x in full-image coordinates, or None if not found in that band.
        """
        total_width = self.x2 - self.x1
        if total_width <= 0:
            return None

        half = band_height // 2
        row_y1 = max(0, y_center - half)
        row_y2 = min(self.image.shape[0], y_center + half)
        if row_y2 <= row_y1:
            return None

        if side == 'left':
            roi = self.source[row_y1:row_y2, self.x1:self.leftZoneX]
        else:
            roi = self.source[row_y1:row_y2, self.rightZoneX:self.x2]
        if roi.size == 0:
            return None

        color_indices = color if isinstance(color, (list, tuple)) else [color]
        blurred = cv2.GaussianBlur(roi, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = None
        for idx in color_indices:
            m = cv2.inRange(hsv, self.lowColor[idx], self.highColor[idx])
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < min_contour_area:
            return None

        xs = largest[:, :, 0].reshape(-1)
        if side == 'left':
            return self.x1 + int(xs.max())   # rightmost pixel of the left zone = inner edge
        else:
            return self.rightZoneX + int(xs.min())   # leftmost pixel of the right zone = inner edge


    def getWallEdgesSplit(self, color=0, scan_height=30, col_threshold=20,
                      contourColor=(0, 255, 0)):

        scan_y1 = max(self.y1, self.y2 - scan_height)
        scan_y2 = self.y2

        total_width = self.x2 - self.x1

        if total_width <= 0:
            return None, None

        mid_col = self.x1 + total_width // 2

        # Left and right boxes
        left_roi = self.source[scan_y1:scan_y2, self.x1:mid_col]
        right_roi = self.source[scan_y1:scan_y2, mid_col:self.x2]

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
