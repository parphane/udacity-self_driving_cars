from os import getcwd, listdir # Listing files
from os.path import isfile, dirname, join # Checking if file is file or directory / building file path
import cv2 # Reading images and images operations
import csv # Images vehicle data
import random # Random image display
import matplotlib.pyplot as plt # All plots


import math # For generic math operations
import numpy as np

import tensorflow as tf
from keras.models import Sequential
from keras.layers import Cropping2D
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D
from keras.layers.advanced_activations import ELU
from keras.regularizers import l2
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint, Callback
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

def main():
    
    EPOCHS = 5
    BATCH_SIZE = 32
    driving_log_path = 'data/driving_log.csv'

    imgs_path, snsrs, lines = load_driving_data(driving_log_path)

    rand_imgs_path, rand_snsrs = augment_snsrs(imgs_path, snsrs)

    augmented_imgs_path = np.concatenate((imgs_path, rand_imgs_path))
    augmented_snsrs = np.concatenate((snsrs, rand_snsrs))
    # Create a flag for randomizing image during training
    augmented_rand_flag = np.concatenate((np.zeros_like(snsrs), np.ones_like(rand_snsrs)))

    shuffled_imgs_path, shuffled_snsrs, shuffled_rand_flag = UnisonShuffle(augmented_imgs_path, augmented_snsrs,
                                                                           augmented_rand_flag)

    X_train, X_test, y_train, y_test, r_train, r_test = \
        train_test_split(shuffled_imgs_path, shuffled_snsrs, shuffled_rand_flag, test_size=0.2, random_state=42,
                         shuffle=True)
    X_train, X_valid, y_train, y_valid, r_train, r_valid = \
        train_test_split(X_train, y_train, r_train, test_size=0.125, random_state=1)

    # Generators for training and validation data
    train_generator = generator(X_train, y_train, r_train, batch_size=BATCH_SIZE)

    valid_generator = generator(X_valid, y_valid, r_valid, batch_size=BATCH_SIZE)

    # Load model
    model = model_nVidia()

    # Train model
    model.compile(loss='mse', optimizer='adam')
    model.fit_generator(train_generator, samples_per_epoch=len(X_train), validation_data=valid_generator,
                        nb_val_samples=len(X_valid), nb_epoch=EPOCHS, verbose=1)

    # Save model
    model.save('model.h5')

def load_driving_data(driving_log_path, header=True, delimiter=',', sa_lr_corr=0.2, verbose=True):
    # Images path is expected to be relative to the driving data directory
    lines = 0
    imgs = None
    snsrs = None

    with open(driving_log_path, 'r') as driving_log_file:
        # Retrieve log directory
        driving_log_dir = dirname(driving_log_path)
        reader = csv.reader(driving_log_file, delimiter=delimiter)
        for line in reader:
            if header:
                header = False
                continue

            lines = lines + 1
            # Center, Left, Right images
            img_cntr_path = join(driving_log_dir, line[0].strip())
            img_left_path = join(driving_log_dir, line[1].strip())
            img_right_path = join(driving_log_dir, line[2].strip())

            # Sensor data: Sensors (snsrs), Steering angle (sa), throttle (thr), brake (brk), speed (spd)
            sa_cntr = float(line[3])

            # Adjusted steering angle for left and right images
            sa_left = min(sa_cntr + sa_lr_corr, 1.0)  # Steer to the right
            sa_right = max(sa_cntr - sa_lr_corr, -1.0)  # Steer to the left

            # Building images and sensors arrays
            imgs_new = np.concatenate(([img_cntr_path], [img_left_path], [img_right_path]))

            snsrs_new = np.concatenate(([sa_cntr], [sa_left], [sa_right]))

            # Image and sensors dataset
            if imgs is None:
                imgs = imgs_new
                snsrs = snsrs_new
            else:
                imgs = np.concatenate((imgs, imgs_new))
                snsrs = np.concatenate((snsrs, snsrs_new))

    return imgs, snsrs, lines


def rand_hsv_channel(img, channel, shift=64):
    rows, cols, _ = img.shape

    # Transform image to HSV to easily manipulate brightness/saturation
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Allow channel shift to be between -shift to +shift V
    d_shift = np.random.randint(-shift, shift)
    hsv[:, :, channel] = cv2.add(hsv[:, :, channel], d_shift)
    dst = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    return dst


def rand_hsv_h(img, shift=64):
    return rand_hsv_channel(img, 0, shift=64)


def rand_hsv_s(img, shift=64):
    return rand_hsv_channel(img, 1, shift=64)


def rand_hsv_v(img, shift=64):
    return rand_hsv_channel(img, 2, shift=64)


def rand_transform(img):
    # Translate, Change perspective (can result in zoom in or out or even rotation), Brightness adjust
    return rand_hsv_h(rand_hsv_s(rand_hsv_v(img)))


def augment_snsrs(imgs_path, snsrs, min_count=750):
    range_step = 5

    # Assuming RGB images for shape
    rand_imgs_path = []
    rand_snsrs = []

    # For steering angle range
    for sa_range in range(-100, 100, range_step):
        # Augment any data up to min_count if necessary
        min_range = sa_range / 100
        max_range = (sa_range + range_step) / 100

        snsrs_range = snsrs
        if (max_range == 1.0):
            # Include 1.0 in the range
            snsrs_range = np.where((snsrs_range >= min_range) & (snsrs_range <= max_range))[0]
        else:
            snsrs_range = np.where((snsrs_range >= min_range) & (snsrs_range < max_range))[0]

        snsrs_count = len(snsrs_range)
        if snsrs_count == 0:
            print("Range [{0:4};{1:4}] is not represented!".format(min_range, max_range))
            continue

        snsrs_augment = min_count - snsrs_count
        if snsrs_augment > 0:
            if (max_range == 1.0):
                print("Range [{0:4};{1:4}] requires {2} augmentations".format(min_range, max_range, snsrs_augment))
            else:
                print("Range [{0:4};{1:4}[ requires {2} augmentations".format(min_range, max_range, snsrs_augment))
            # Add as many augmented images as necessary
            for i in range(snsrs_augment):
                # Pick images one after the other, and loop if necessary
                rand_imgs_path.append(imgs_path[snsrs_range[i % snsrs_count]])
                rand_snsrs.append(snsrs[snsrs_range[i % snsrs_count]])
        else:
            if (max_range == 1.0):
                print("Range [{0:4};{1:4}[ does not require augmentation".format(min_range, max_range))
            else:
                print("Range [{0:4};{1:4}[ does not require augmentation".format(min_range, max_range))
            pass

    return rand_imgs_path, rand_snsrs


def UnisonShuffle(a, b, c):
    assert len(a) == len(b)
    assert len(a) == len(c)
    p = np.random.permutation(len(a))
    return a[p], b[p], c[p]

def gaussian_blur(img, ksize=3):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (ksize, ksize), 0)

def rgb_2_gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

def normalize_gray(imgs, max=255):
    # Subtract half the max (256) value to center around 0 then divide by the same value to obtain a [-1;1] range
    half = max/2.0
    return (imgs - half)/half


def model_nVidia():
    model = Sequential()

    # Crop irrelevant data from FOV
    model.add(Cropping2D(cropping=((70, 25), (0, 0)), input_shape=(160, 320, 3)))

    # Normalize and center (using lambda) (after crop to save resources)
    model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160, 320, 3)))

    # Nvidia model
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), activation='relu'))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), activation='relu'))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), activation='relu'))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(Convolution2D(64, 3, 3, activation='relu'))
    model.add(Flatten())
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))

    return model


def generator(samples_paths, samples_angles, samples_rand_flags, batch_size):
    samples_count = len(samples_paths)

    while True:
        # Samples are already shuffled
        # sklearn.utils.shuffle(samples)

        # For each batch sized slice available from the samples
        for offset in range(0, samples_count, batch_size):
            # Retrieve batch samples
            batch_paths = samples_paths[offset:offset + batch_size]
            batch_angles = samples_angles[offset:offset + batch_size]
            batch_rand_flags = samples_rand_flags[offset:offset + batch_size]

            images = []
            angles = []

            # For each sample in the batch
            for i in range(0, len(batch_paths)):
                # Read image
                image = cv2.cvtColor(cv2.imread(batch_paths[i]), cv2.COLOR_BGR2RGB)
                # Apply random transformations if flagged
                if (batch_rand_flags[i] > 0):
                    image = rand_transform(image)
                images.append(image)
                angles.append(batch_angles[i])

            # Transform arrays to np arrays
            X = np.array(images)
            y = np.array(angles)
            yield X, y
    return

if __name__ == '__main__':
    main()