#!/usr/bin/env python3.8

import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
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
from mogi_chess_msgs.srv import SaveFenSamples, SaveFenSamplesResponse, ReadLastMove, ReadLastMoveResponse

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

rospy.init_node('detect_pieces')
param_sim = rospy.get_param('~sim', "false")
if param_sim:
    model_name = "model_simple_sim.best.h5"
else:
    model_name = "model_simple.best.h5"

rospack = rospkg.RosPack()

path = rospack.get_path('mogi_chess_vision')
save_path = path + "/tmp/"

model_path = path + "/train/" + model_name
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
        self.inference_trigger = False
        self.result_frame = None
        self.previous_tracked_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        self.previous_side = None
        self.last_white_move = None

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

        if self.result_frame is None:
            self.result_frame = img.copy()
        

        if self.inference_trigger:

            self.result_frame = img.copy()

            rows,cols = img.shape[:2]
            row_height = int((rows - padding_top - padding_bottom) / 8)
            col_width = int((cols - padding_left - padding_right) / 8)

            squares = []
            for i in range(0,8):
                for j in range(0,8):
                    square = img[(padding_left - square_margin + i * col_width):(padding_left + square_margin + (i + 1) * col_width), (padding_top - square_margin + j * row_height):(padding_top + square_margin + (j + 1) * row_height)]
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

                #cv2.putText(self.result_frame, label,
                #        (60 + (i % 8) * col_width, 80 + j * row_height),
                #        cv2.FONT_HERSHEY_SIMPLEX,
                #        0.6, (0, 0, 255), 2)

            print("Batch inference time: %.3f" % (time.perf_counter()-start_time))
            #print(fen_input)
            fen = helper_lib.get_fen(fen_input)
            print("CNN Fen: %s" % fen)

            #cv2.putText(self.result_frame, fen,
            #            (50, rows - 10),
            #            cv2.FONT_HERSHEY_SIMPLEX,
            #            1, (0, 0, 255), 3)

            new_tracked_fen, move = self.track_fen(self.previous_tracked_fen, fen)

            if new_tracked_fen == "invalid":
                new_tracked_fen = self.previous_tracked_fen
                print("Invalid movement!")
                self.last_white_move = "invalid"
            else:
                self.previous_tracked_fen = new_tracked_fen
                print(f"Tracked Fen: {new_tracked_fen}, move: {move}")

                if move == "invalid":
                    self.last_white_move = "invalid"
                elif new_tracked_fen.split(" ")[1] == "b" and move != "invalid":
                    self.last_white_move = move

            tracked_pieces = helper_lib.get_list_fom_fen(new_tracked_fen)
            print(tracked_pieces)
            j = -1
            for i, piece in enumerate(tracked_pieces):
                if i % 8 == 0:
                    j+=1

                piece = helper_lib.short2label(piece)

                cv2.putText(self.result_frame, piece,
                        (60 + (i % 8) * col_width, 80 + j * row_height),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 0, 255), 2)


            self.inference_trigger = False


        return self.result_frame

    def track_fen(self, prev_fen, cnn_output):

        fen, move = helper_lib.track_fen(prev_fen, cnn_output)

        return fen, move

    def serve_track_fen(self, req):
        
        self.fen = req.fen.split()[0]
        print(f"True FEN: {self.fen}")
        self.inference_trigger = True

        while self.inference_trigger:
            time.sleep(0.1)

        if self.previous_tracked_fen.split(" ")[0] == self.fen:
            print("FEN matches!")
        else:
            print("Oh no! FEN doesn't match!")
            raise Exception("ajjaj")

        #input("\npress enter to continue\n")
        return SaveFenSamplesResponse(True)

    def clock_handler(self, msg):

        if self.previous_side != msg.data:
            self.previous_side = msg.data

            self.inference_trigger = True

            while self.inference_trigger:
                time.sleep(0.1)

    def serve_last_move(self, req):

        return ReadLastMoveResponse(self.last_white_move)


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

# Define your image topic
image_topic = "/chessboard_image/color/image_raw"

rospy.Subscriber(image_topic, Image, queueMonocular)
rospy.Subscriber("/mogi_chess_clock/side", String, cvThreadHandle.clock_handler)
s_track = rospy.Service('save_fen_samples', SaveFenSamples, cvThreadHandle.serve_track_fen)
s_last_mive = rospy.Service('read_last_move', ReadLastMove, cvThreadHandle.serve_last_move)

# Spin until Ctrl+C
rospy.spin()