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
import tensorflow as tf
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
import h5py
from tensorflow.keras import __version__ as keras_version
import numpy as np
import helper_lib
import os

#os.environ['CUDA_VISIBLE_DEVICES'] = '-1'
#if tf.test.gpu_device_name():
#    print('GPU found')
#else:
#    print("No GPU found")

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

padding_left = 50
padding_right = 50
padding_top = 50
padding_bottom = 50
# margin has to be less or equal to padding_left or padding_top
square_margin = 50
image_size = 100

rospack = rospkg.RosPack()

path = rospack.get_path('mogi_chess_vision')
save_path = path + "/tmp/"

model_path = path + "/train/model.best.h5"
f = h5py.File(model_path, mode='r')
model_version = f.attrs.get('keras_version')
keras_version = str(keras_version).encode('utf8')

if model_version != keras_version:
    print('You are using Keras version ', keras_version,
            ', but the model was built using ', model_version)

model = load_model(model_path)

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
        self.save_delay = 5.0
        self.save_delay = float("inf")
        self.last_save_time = 0
        self.save_this = False

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
        result_frame = img.copy()
        # save
        if time.time() > self.last_save_time + self.save_delay:
            self.save_this = True
            time_postfix = datetime.today().strftime('%Y%m%d-%H%M%S-%f')
            cv2.imwrite(save_path + "board-" + time_postfix + ".jpg", img)
            square_path = save_path + time_postfix
            os.mkdir(square_path)

        rows,cols = img.shape[:2]
        row_height = int((rows - padding_top - padding_bottom) / 8)
        col_width = int((cols - padding_left - padding_right) / 8)

        squares = []
        for i in range(0,8):
            for j in range(0,8):
                square = img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
                if self.save_this:
                    cv2.imwrite(square_path + "/square-" + str(i) + "-" + str(j) + "-" + time_postfix + ".jpg", square)
                square = cv2.resize(square, (image_size, image_size))
                square = img_to_array(square)
                square = np.array(square, dtype="float") / 255.0
                squares.append(square)

        start_time = time.perf_counter()
        predictions = model.predict_on_batch(np.asarray(squares))
        j = -1
        fen_input = []
        for i, pred_i in enumerate(predictions):
            if i % 8 == 0:
                j+=1

            prediction = np.argmax(pred_i)
            label, short = helper_lib.class2label(prediction)
            #print(label, short)
            fen_input.append(short)

            cv2.putText(result_frame, label,
                    (60 + (i % 8) * col_width, 80 + j * row_height),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 255), 2)

        print("Batch inference time: %.3f" % (time.perf_counter()-start_time))
        #print(fen_input)
        fen = helper_lib.get_fen(fen_input)
        print("Fen: %s" % fen)

        cv2.putText(result_frame, fen,
                    (50, rows - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)


        if self.save_this:
            cv2.imwrite(save_path + "result-" + time_postfix + ".jpg", result_frame)
            print("Images were saved!")
            self.last_save_time = time.time()
            self.save_this = False

        return result_frame


def split_depth(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)


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
#depth_topic = "/chessboard_image/depth/image_raw"

rospy.Subscriber(image_topic, Image, queueMonocular)
#rospy.Subscriber(depth_topic, Image, split_depth)

# Spin until Ctrl+C
rospy.spin()