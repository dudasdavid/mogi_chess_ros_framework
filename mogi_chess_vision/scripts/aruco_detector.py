#!/usr/bin/env python3.8

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np

selected_type = "DICT_4X4_50"

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
if ARUCO_DICT.get(selected_type, None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(
        args["type"]))
    sys.exit(0)
# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(selected_type))
arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[selected_type])
arucoParams = cv2.aruco.DetectorParameters_create()

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, image_queue, depth_queue):
        threading.Thread.__init__(self)
        self.image_queue = image_queue
        self.depth_queue = depth_queue
        self.image = None
        self.depth = None
        self.id1_location = None
        self.id2_location = None
        self.id3_location = None
        self.id4_location = None

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.image_queue.get()
            rows,cols = self.image.shape[:2]
            if depth_enable:
                self.depth = self.depth_queue.get()
            else:
                self.depth = np.zeros((rows,cols,3), np.uint8)

            # Process the current image
            detection, rejected, warped, warped_depth = self.processImage(self.image)

            # Add processed images as small images on top of main image
            result = self.addSmallPictures(self.image, [detection, rejected, warped, self.depth, warped_depth])
            cv2.imshow("frame", result)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')

    def processImage(self, img):

        self.id1_location = None
        self.id2_location = None
        self.id3_location = None
        self.id4_location = None

        detection_frame = img.copy()
        rejected_frame = img.copy()
        rows,cols = img.shape[:2]

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

        cv2.aruco.drawDetectedMarkers(rejected_frame, rejected)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # draw the bounding box of the ArUCo detection
                cv2.line(detection_frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(detection_frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(detection_frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(detection_frame, bottomLeft, topLeft, (0, 255, 0), 2)
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(detection_frame, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the frame
                cv2.putText(detection_frame, str(markerID),
                    (topLeft[0], topLeft[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

                if markerID == 1:
                    self.id1_location = [cX, cY]
                elif markerID == 2:
                    self.id2_location = [cX, cY]
                elif markerID == 3:
                    self.id3_location = [cX, cY]
                elif markerID == 4:
                    self.id4_location = [cX, cY]

        if self.id1_location is not None and self.id2_location is not None and self.id3_location is not None and self.id4_location is not None:
            warped = self.image_transform(img, [self.id1_location, self.id2_location, self.id3_location, self.id4_location])
            chessboard_publisher.publish(bridge.cv2_to_imgmsg(warped, "bgr8"))

            if depth_enable:
                warped_depth = self.image_transform(self.depth, [self.id1_location, self.id2_location, self.id3_location, self.id4_location])
                chessboard_depth_publisher.publish(bridge.cv2_to_imgmsg(warped_depth, "bgr8"))
            else:
                warped_depth = np.zeros((rows,cols,3), np.uint8)
        else:
            warped = np.zeros((rows,cols,3), np.uint8)
            warped_depth = np.zeros((rows,cols,3), np.uint8)

        # Return processed frames
        return detection_frame, rejected_frame, warped, warped_depth

    def image_transform(self, img, points):
        """Crop original image using perspective warp."""
        board_length = 1200

        pts1 = np.float32(points)
        pts2 = np.float32([[0, board_length], [board_length, board_length],
                        [board_length, 0], [0, 0]])
        mat = cv2.getPerspectiveTransform(pts1, pts2)
        return cv2.warpPerspective(img, mat, (board_length, board_length))

    # Add small images to the top row of the main image
    def addSmallPictures(self, img, small_images, size=(213, 160)):
        '''
        :param img: main image
        :param small_images: array of small images
        :param size: size of small images
        :return: overlayed image
        '''

        x_base_offset = 20
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

def queueDepth(msg):
    global depth_enable
    depth_enable = True
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        img_n = cv2.normalize(src=cv2Img, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        im_color = cv2.cvtColor(img_n, cv2.COLOR_GRAY2BGR)
    except CvBridgeError as e:
        print(e)
    else:
        qDepth.put(im_color)

print("Python version: %s" % sys.version)
print("OpenCV version: %s" % cv2.__version__)

depth_enable = False
queueSize = 1      
qMono = BufferQueue(queueSize)
qDepth = BufferQueue(queueSize)

cvThreadHandle = cvThread(qMono, qDepth)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

bridge = CvBridge()

rospy.init_node('aruco_detector')
# Define your image topic
image_topic = "/camera/color/image_raw"
depth_topic = "/camera/aligned_depth_to_color/image_raw"
# Set up your subscriber and define its callback
rospy.Subscriber(image_topic, Image, queueMonocular)
rospy.Subscriber(depth_topic, Image, queueDepth)
chessboard_publisher = rospy.Publisher("/chessboard_image/color/image_raw", Image, queue_size=1)
chessboard_depth_publisher = rospy.Publisher("/chessboard_image/depth/image_raw", Image, queue_size=1)


# Spin until Ctrl+C
rospy.spin()