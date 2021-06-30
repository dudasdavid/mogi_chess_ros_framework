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
from mogi_chess_msgs.srv import SaveFenSamples, SaveFenSamplesResponse

#import tensorflow as tf
#from tensorflow.keras.preprocessing.image import img_to_array
#from tensorflow.keras.models import load_model
#import h5py
#from tensorflow.keras import __version__ as keras_version
#import numpy as np
import helper_lib
#import os

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
        self.save_this = False
        self.fen = None

    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("marked_image", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("marked_image", 1200,1200)

        while True:
            self.image = self.image_queue.get()

            marked_image = self.split_image(self.image)
            cv2.imshow("marked_image", marked_image)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')

    def split_image(self, img):
        # save
        if self.save_this:
            time_postfix = datetime.today().strftime('%Y%m%d-%H%M%S-%f')

        rows,cols = img.shape[:2]
        row_height = int((rows - padding_top - padding_bottom) / 8)
        col_width = int((cols - padding_left - padding_right) / 8)

        for i in range(0,8):
            for j in range(0,8):
                square = img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
                if self.save_this:
                    if self.fen != None:
                        piece = helper_lib.get_piece_ij(self.fen, i, j)
                        print(piece, i, j)
                    else:
                        print("FEN was None")
                    cv2.imwrite(save_path + "/" + piece + "/" + piece + "-" + str(i) + "-" + str(j) + "-" + time_postfix + ".jpg", square)

        if self.save_this:
            print("Images were saved!")
            self.save_this = False

        return img

    def serve_save_fen_samples(self, req):
        self.fen = req.fen.split()[0]
        print(self.fen)
        self.save_this = True
        while self.save_this:
            time.sleep(0.1)

        return SaveFenSamplesResponse(True)


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

rospy.init_node('detect_pieces')
# Define your image topic
image_topic = "/chessboard_image/color/image_raw"
param_sim = rospy.get_param('~sim', "false")
if param_sim:
    save_path = path + "/samples_sim/"
else:
    save_path = path + "/samples/"

rospy.Subscriber(image_topic, Image, queueMonocular)

s_save = rospy.Service('save_fen_samples', SaveFenSamples, cvThreadHandle.serve_save_fen_samples)

# Spin until Ctrl+C
rospy.spin()