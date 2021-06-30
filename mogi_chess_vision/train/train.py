# import the necessary packages
from tensorflow.keras.models import Sequential, load_model
from tensorflow.keras.layers import Activation, Flatten, Dense, Dropout, Conv2D, MaxPooling2D, Lambda, BatchNormalization
from tensorflow.keras.callbacks import ReduceLROnPlateau, ModelCheckpoint
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.optimizers import Adam, SGD
from tensorflow.keras.losses import SparseCategoricalCrossentropy
from tensorflow.keras.metrics import CategoricalAccuracy
from sklearn.model_selection import train_test_split
import sklearn
from imutils import paths
import numpy as np
import random
import cv2
import os
import matplotlib.pyplot as plt
from argparse import ArgumentParser
import math
import helper_lib

from numpy.random import seed
from tensorflow.random import set_seed

from tensorflow.compat.v1 import ConfigProto
from tensorflow.compat.v1 import InteractiveSession

config = ConfigProto()
config.gpu_options.allow_growth = True
session = InteractiveSession(config=config)

# Fix every random seed to make the training reproducible
seed(1)
set_seed(2)
random.seed(42)

image_size = 100

def build_argparser():
    # parse command line arguments.
    parser = ArgumentParser()
    parser.add_argument("-i", "--input", required=False, type=str, default="model.best.h5",
                        help="Path to model")
    parser.add_argument("-c", "--is_continue", type=bool, default=False,
                        help="Continue training")
    parser.add_argument("-s", "--use_sim", type=bool, default=False,
                        help="Use simulation folder")            

    return parser


def build_model(width, height, depth, classes):
    # initialize the model
    model = Sequential()
    input_shape = (height, width, depth)

    # normalization with lambda layer
    #model.add(Lambda(lambda x: x / 127.5 - 1., input_shape=input_shape))

    # first set of CONV => RELU => POOL layers
    model.add(Conv2D(20, (5, 5), padding="same", input_shape=input_shape))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    
    # second set of CONV => RELU => POOL layers
    model.add(Conv2D(50, (5, 5), padding="same"))
    model.add(Activation("relu"))
    model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    
    # first (and only) set of FC => RELU layers
    model.add(Flatten())
    model.add(Dropout(0.2))
    model.add(Dense(500))
    model.add(Activation("relu"))

    # softmax classifier
    model.add(Dense(classes))
    model.add(Activation("softmax"))
    
    # return the constructed network architecture
    return model
   
# grab command line args
args = build_argparser().parse_args()

if not args.use_sim:
    dataset = '../samples'
else:
    dataset = './samples_sim'

# initialize the data and labels
print("[INFO] loading images...")
data = []
labels = []
 
# grab the image paths and randomly shuffle them
samples = sorted(list(paths.list_images(dataset)))
random.shuffle(samples)

# split training and validation samples
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

# sample generator
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        random.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            labels = []
            for batch_sample in batch_samples:
                # print(batch_sample)
                image = cv2.imread(batch_sample)
                #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # OpenCV reads in BGR format
                image = cv2.resize(image, (image_size, image_size))
                image = img_to_array(image)
                images.append(image)
                label = batch_sample.split(os.path.sep)[-2]
                #print(label)
                label = helper_lib.label2class(label)
                #print(label)
                labels.append(label)

                """
                # horizontal flip center image
                images.append(cv2.flip(image, 1))
                label = batch_sample.split(os.path.sep)[-2]
                #print(label)
                label = helper_lib.label2class(label)
                labels.append(label)
                """

            X_train = np.array(images, dtype="float") / 255.0
            y_train = np.array(labels)
            yield tuple(sklearn.utils.shuffle(X_train, y_train))

# set our batch size
batch_size=16
# create generator functions for training and validation samples
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

# hyperparameters
EPOCHS  = 12
INIT_LR = 0.01
DECAY   = INIT_LR / EPOCHS

# initialize a new model
if not args.is_continue:
    model = build_model(width=image_size, height=image_size, depth=3, classes=13)

    opt = SGD(lr=INIT_LR, decay=DECAY)
    model.compile(optimizer=opt, loss=SparseCategoricalCrossentropy(from_logits=True), metrics=["accuracy"])#, CategoricalAccuracy()])
# continue the training of exisiting model
else:
    model = load_model(args.input)

# print model summary
model.summary()

# set a learning rate annealer
reduce_lr = ReduceLROnPlateau(monitor='val_loss',
                                patience=3,
                                verbose=1,
                                factor=0.5,
                                min_lr=1e-6)

# checkpoint the best model
if not args.use_sim:
    filepath = "model.best.h5"
else:
    filepath = "model_sim.best.h5"

checkpoint = ModelCheckpoint(filepath, monitor = 'val_loss',verbose=1,
                             save_best_only=True,mode='min')

# callbacks
callbacks_list=[reduce_lr, checkpoint]

# use fit generator instead of fit method
history = model.fit(train_generator, steps_per_epoch=math.ceil(len(train_samples)/batch_size), epochs=EPOCHS, validation_data=validation_generator, validation_steps=math.ceil(len(validation_samples)/batch_size), callbacks=callbacks_list, verbose=1)

# save the model
if not args.use_sim:
    model.save("model.h5")
else:
    model.save("model_sim.h5")

# plot and save training history
plt.xlabel('Epoch Number')
plt.ylabel("Loss (MSE) Magnitude")
plt.plot(history.history['loss'], label="loss")
plt.plot(history.history['val_loss'], label="val_loss")
plt.plot(history.history['accuracy'], label="accuracy")
plt.plot(history.history['val_accuracy'], label="val_accuracy")
#plt.plot(history.history['categorical_accuracy'], label="categorical_accuracy")
#plt.plot(history.history['val_categorical_accuracy'], label="val_categorical_accuracy")
plt.legend()
if not args.use_sim:
    plt.savefig('model_training')
else:
    plt.savefig('model_training_sim')

plt.close()
