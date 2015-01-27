"""This module contains class for identifying vision targets.

NOTE: THIS RUNS ON THE DRIVER STATION, NOT ON THE ROBOT.

DO NOT UPLOAD TO THE ROBOT!!

"""

import cv2
import logging
import math
import numpy as np
import sys
import target
import urllib2


class ContourInfo(object):
    """Data structure to store details of a contour."""
    contour = None  # The opencv contour that is the basis for this data
    bounding_rect = None    # The bounding rectangle for this contour
    actual_width = None # The actual width of this contour (even if rotated)
    actual_height = None    # The actual height of this contour
    contour_area = None # The actual contour area
    bounding_area = None    # The area of the bounding rectangle
    is_vertical = None  # True if the contour is taller than it is wide
    rectangularity = None   # The rectangularity score for the contour
    center_mass = None  # The x,y position of the contour's center of mass
    aspect_ratio = None # The aspect ratio score of the contour
    paired_horizontal_contour_data = None   # The paired horizontal contour
    pairing_distance = None # The distance between this contour and the paired
    pairing_score = None    # The pairing score of the horizontal contour
    pairing_vertical_score = None   # The vertical score of the paired contour


class Targeting(object):
    """Gets images from a webcam and finds Targets using opencv."""
    # TODO: convert to read these from a parameter file
    # TODO: search the rest of the code for constants

    # Camera view angle (49 for 1013)
    #CAMERA_URL = ("http://10.0.94.11/axis-cgi/mjpg/video.cgi?"
    #              "resolution=640x480&dummy=param.mjpg")
    #CAMERA_URL = r"http://10.0.94.11/mjpg/video.mjpg"
    CAMERA_URL = r"http://10.0.94.11/jpg/image.jpg"
    CAMERA_VIEW_ANGLE = 49
    CAMERA_RES_HEIGHT = 640
    CAMERA_RES_WIDTH = 480
    GREEN_MIN = np.array([75, 160, 65], np.uint8)
    GREEN_MAX = np.array([92, 255, 180], np.uint8)
    RECTANGULARITY_THRESHOLD = 40
    ASPECT_RATIO_THRESHOLD = 55

    _logger = None

    #_vcap = None

    #def __init__(self):
        #"""Create a Targeting object and Video Capture for the camera."""
        #self._vcap = cv2.VideoCapture()

    def __init__(self, log_handler=None):
        self._logger = logging.getLogger(__name__)
        handler = None
        if log_handler:
            handler = log_handler
        else:
            formatter = logging.Formatter('%(asctime)s - %(levelname)s:'
                                          '%(name)s:%(message)s')
            handler = logging.StreamHandler(stream=sys.stdout)
            handler.setLevel(logging.DEBUG)
            handler.setFormatter(formatter)
        self._logger.addHandler(handler)
        self._logger.setLevel(logging.DEBUG)

    def open(self):
        """Try opening a connection to the camera."""
        #"""Try to open the Video Capture object linked to the camera."""
        #return self._vcap.open(self.CAMERA_URL)
        retval = False
        try:
            stream = urllib2.urlopen(self.CAMERA_URL, None, 1)
            stream.close()
            retval = True
        except Exception as excep:
            self._logger.error("Exception connecting to camera: " + str(excep))
        return retval

    #def close(self):
        #"""Close the Video Capture object."""
        #self._vcap.release()

    def get_image(self):
        """Get the latest frame."""
        #if self._vcap:
        #    result, img = self._vcap.read()
        #    if result:
        #        return img
        #else:
        #    return None
        #return cv2.imread('input.jpg')
        img = None
        try:
            stream = urllib2.urlopen(self.CAMERA_URL, None, 1)
            data = stream.read()
            stream.close()
            img = cv2.imdecode(np.fromstring(data, dtype=np.uint8),
                               cv2.CV_LOAD_IMAGE_COLOR)
        except Exception as excep:
            self._logger.error("Exception getting image: " + str(excep))
        return img

    def score_aspect_ratio(self, contour_data):
        """Score the aspect ratio of a contour based on expected sizes."""
        rect_x, rect_y, rect_w, rect_h = contour_data.bounding_rect
        ideal_ratio = (4.0 / 32.0) if contour_data.is_vertical else (23.5 / 4.0)

        rect_long = rect_w if rect_w > rect_h else rect_h
        rect_short = rect_w if rect_w < rect_h else rect_h

        ratio = None
        if rect_w > rect_h:
            ratio = (rect_long * 1.0 / rect_short) / ideal_ratio
        else:
            ratio = (rect_short * 1.0 / rect_long) / ideal_ratio

        return max(0, min(100 * (1 - math.fabs(1 - ratio)), 100))

    def calculate_pairing_score(self, ratio):
        """Converts a pairing ratio into a score.

        The pairing ratio involves the distance between the vertical and
        horizontal contours being compared relative to their sizes.

        """
        if ratio <= 0 or ratio > 2:
            return 0.0
        return 100.0 - (math.fabs(1.0 - ratio) * 100.0)

    def calculate_distance(self, v_contour_data):
        """Calculate the distance to the contour from the robot."""
        distance = 0
        try:
            rect_x, rect_y, rect_w, rect_h = v_contour_data.bounding_rect
            rect_long = rect_w if rect_w > rect_h else rect_h
            height = min(rect_h, rect_long)
            target_height = 32
            distance = (self.CAMERA_RES_WIDTH * target_height /
                       (height * 12 * 2 *
                        math.tan(self.CAMERA_VIEW_ANGLE * math.pi / (180 * 2))))
        except Exception as excep:
            self._logger.error("Exception calculating distance: " + str(excep))
        return distance

    def calculate_angle(self, v_contour_data):
        """Calculate the degrees off target between the robot and target."""
        v_center_x, v_center_y = v_contour_data.center_mass
        degrees_per_pixel = (85 / math.sqrt((self.CAMERA_RES_HEIGHT ** 2) +
                                            (self.CAMERA_RES_WIDTH ** 2)))
        pixels_off_center = v_center_x - (self.CAMERA_RES_WIDTH / 2)
        return pixels_off_center * degrees_per_pixel

    def is_valid_target(self, contour_data):
        """Compare contour scores against minimum thresholds."""
        return (contour_data.rectangularity > self.RECTANGULARITY_THRESHOLD and
                contour_data.aspect_ratio > self.ASPECT_RATIO_THRESHOLD)

    def get_targets(self):
        """Get an image, search it for targets, and return a list of Targets."""
        img = self.get_image()
        if img == None:
            return []

        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.cv.CV_BGR2HSV)

        threshold = cv2.inRange(hsv, self.GREEN_MIN, self.GREEN_MAX)
        #cv2.imwrite("threshold.png", threshold)

        # Dilate
        element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilate = cv2.dilate(threshold, element, iterations=1)
        #cv2.imwrite("dilate.png", dilate)

        # Erode
        erode = cv2.erode(dilate, element, iterations=1)
        #cv2.imwrite("erode.png", erode)

        # Fill in the gaps
        # This actually seems to mess up the filtering, maybe since we dilate?
        #kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,2), anchor=(1,1))
        #morphed = cv2.morphologyEx(erode, cv2.MORPH_CLOSE, kernel,iterations=9)
        #cv2.imwrite("morphed.png", morphed)

        # Find contours
        contours, hierarchy = cv2.findContours(erode.copy(),
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_TC89_KCOS)

        # Only used for saving resulting image
        #color_img = cv2.cvtColor(erode, cv2.cv.CV_GRAY2BGR)

        # Calculate basic information about each contour
        contours_data = []
        for contour in contours:
            current_contour = ContourInfo()
            current_contour.contour = contour
            current_contour.contour_area = cv2.contourArea(contour)
            ((center_x, center_y), (width, height), angle) = cv2.minAreaRect(
                                                                        contour)
            current_contour.actual_width = width
            current_contour.actual_height = height
            rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(contour)
            current_contour.bounding_rect = (rect_x, rect_y, rect_w, rect_h)
            current_contour.bounding_area = rect_w * rect_h
            center_x = rect_x + (rect_w / 2)
            center_y = rect_y + (rect_h / 2)
            current_contour.center_mass = (center_x, center_y)
            ratio = (rect_w * 1.0) / rect_h
            if ratio > 1.0:
                current_contour.is_vertical = False
            else:
                current_contour.is_vertical = True
            current_contour.rectangularity = ((current_contour.contour_area /
                                               current_contour.bounding_area) *
                                              100.0)
            current_contour.aspect_ratio = self.score_aspect_ratio(
                                                                current_contour)
            contours_data.append(current_contour)

        # Split contours by vertical/horizontal and remove invalid targets
        vertical_contours = []
        horizontal_contours = []
        for contour_data in contours_data:
            if self.is_valid_target(contour_data):
                if contour_data.is_vertical:
                    vertical_contours.append(contour_data)
                else:
                    horizontal_contours.append(contour_data)

        # Draw each contour
        #for contour_data in vertical_contours:
        #    cv2.drawContours(color_img, contour_data.contour, -1, (0,0,255),
        #                     thickness=2)

        #for contour_data in horizontal_contours:
        #    cv2.drawContours(color_img, contour_data.contour, -1, (0,0,255),
        #                     thickness=2)

        # Check each vertical contour for a matching horizontal contour
        matched_contours_data = []
        for v_contour_data in vertical_contours:
            v_rect_x, v_rect_y, v_rect_w, v_rect_h = v_contour_data.\
                                                                bounding_rect
            pair_found = False
            for h_contour_data in horizontal_contours:
                h_rect_x, h_rect_y, h_rect_w, h_rect_h = h_contour_data.\
                                                                bounding_rect
                h_center_x, h_center_y = h_contour_data.center_mass
                dist = cv2.pointPolygonTest(v_contour_data.contour,
                                            h_contour_data.center_mass, True)
                pairing_ratio = (math.fabs(dist) / h_rect_w)
                pairing_score = self.calculate_pairing_score(pairing_ratio)
                vertical_score = (1.0 - (math.fabs(v_rect_y - h_center_y) /
                                         (4.0 * h_rect_h)))
                if vertical_score > 0.8 and pairing_score > 50:
                    v_contour_data.paired_horizontal_contour_data = \
                                                                h_contour_data
                    v_contour_data.pairing_distance = dist
                    v_contour_data.pairing_score = pairing_score
                    v_contour_data.pairing_vertical_score = vertical_score
                    matched_contours_data.append(v_contour_data)
                    pair_found = True
                    break
            if not pair_found:
                matched_contours_data.append(v_contour_data)

        # Turn each vertical/horizontal contour into a Target
        targets = []
        for v_contour_data in matched_contours_data:
            current_target = target.Target()
            current_target.no_targets = False
            current_target.angle = self.calculate_angle(v_contour_data)
            current_target.distance = self.calculate_distance(v_contour_data)
            if v_contour_data.paired_horizontal_contour_data:
                current_target.is_hot = True
                current_target.confidence = (
                                        v_contour_data.pairing_vertical_score *
                                        50.0 + v_contour_data.pairing_score /
                                        2.0)
                side = (v_contour_data.bounding_rect[0] -
                        v_contour_data.paired_horizontal_contour_data.\
                                                            bounding_rect[0])
                if side < 0:
                    current_target.side = target.Side.RIGHT
                else:
                    current_target.side = target.Side.LEFT
            else:
                current_target.confidence = 0.0
                current_target.is_hot = False
                current_target.side = target.Side.UNKNOWN
            targets.append(current_target)

        #cv2.imwrite("result.png", color_img)
        return targets
