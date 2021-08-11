#!/usr/bin/env python3.8

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import rospy
import rospkg 
import time
from datetime import datetime

try:
    from queue import Queue
except ImportError:
    from Queue import Queue

import threading

padding_left = 50
padding_right = 50
padding_top = 50
padding_bottom = 50
# margin has to be less or equal to padding_left or padding_top
square_margin = 50

rospack = rospkg.RosPack()

path = rospack.get_path('mogi_chess_vision')

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
    def __init__(self, image_queue):
        threading.Thread.__init__(self)
        self.image_queue = image_queue
        self.image = None

    def run(self):

        self.image = self.image_queue.get()

        self.split_and_save(self.image)

        rospy.signal_shutdown('Quit')

    def split_and_save(self, img):
        # save
        time_postfix = datetime.today().strftime('%Y%m%d-%H%M%S-%f')

        rows,cols = img.shape[:2]
        row_height = int((rows - padding_top - padding_bottom) / 8)
        col_width = int((cols - padding_left - padding_right) / 8)

        for i in range(0,8):
            for j in range(0,8):
                square = img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
                cv2.imwrite(save_path + "/" + str(i) + "-" + str(j) + "-" + time_postfix + ".jpg", square)

        return


def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

print("Python version: %s" % sys.version)
print("OpenCV version: %s" % cv2.__version__)

queueSize = 1   
qMono = BufferQueue(queueSize)

cvThreadHandle = cvThread(qMono)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

bridge = CvBridge()

rospy.init_node('save_snapshots')
# Define your image topic
image_topic = "/chessboard_image/color/image_raw"
param_sim = rospy.get_param('~sim', "false")
if param_sim:
    save_path = path + "/extra_samples_sim/"
else:
    save_path = path + "/extra_samples/"

rospy.Subscriber(image_topic, Image, queueMonocular)

# Spin until Ctrl+C
rospy.spin()