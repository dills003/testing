import csv
import cv2
import numpy as np
import tensorflow as tf
import sklearn
from sklearn.model_selection import train_test_split


lines = []

with open("C:\\Users\\dills\\Desktop\\Data\\driving_log.csv") as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)
        

train_samples, validation_samples = train_test_split(lines, test_size=0.2)
train_samplesLen = len(train_samples) * 3 * 2

print(train_samplesLen)

def generator(samples, batch_size=100):
    num_samples = len(samples)
    while 1:
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            measurements = []
            steeringCorrection = .625

            for line in batch_samples:
                for i in range(3): #new
                    source_path = line[i]#changed from 0 to i
                    image = cv2.imread(source_path)
                    images.append(image)
                    measurement = float(line[3])
                    if i == 0:
                        measurements.append(measurement)
                    elif i == 1:
                        measurements.append(measurement + steeringCorrection)
                    elif i == 2:
                        measurements.append(measurement - steeringCorrection)


            aug_images = []
            aug_measurements = []
            for image, measurement in zip(images, measurements):
                aug_images.append(image)
                aug_measurements.append(measurement)
                aug_images.append(cv2.flip(image, 1))
                aug_measurements.append(measurement*(-1.0))

            X_train = np.array(aug_images)
            y_train = np.array(aug_measurements)
            yield sklearn.utils.shuffle(X_train, y_train)

train_generator = generator(train_samples, batch_size=100)
validation_generator = generator(validation_samples, batch_size=100)


from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D


model = Sequential()
model.add(Cropping2D(cropping=((80,25),(0,0)),input_shape=(160,320,3)))
model.add(Lambda(lambda x: x / (255.0 - 0.5)))
model.add(Convolution2D(12,5,5,subsample=(2,2),activation="relu"))
model.add(MaxPooling2D())
model.add(Convolution2D(36,5,5,subsample=(2,2),activation="relu"))
model.add(MaxPooling2D())
model.add(Flatten())
model.add(Dense(200,activation="sigmoid"))
model.add(Dense(100))#100
model.add(Dense(50))#100
model.add(Dense(1))#1
model.compile(loss='mse', optimizer='adam')
model.fit_generator(train_generator,samples_per_epoch=train_samplesLen,validation_data=validation_generator,nb_val_samples=len(validation_samples), nb_epoch=10)

model.save('model.h5')


