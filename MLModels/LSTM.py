# Webots.HPC Project
# LSTM Deep-Learning Model
# Matt Franchi


# MODULE IMPORTS
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt


# DATA PROCESSING
FILENAME = "data_1621354319_565439.csv"
data = np.genfromtxt("../output/"+FILENAME,dtype=float,delimiter=',',names=True)
print(data[:10])

# MODEL CONSTRUCTION
def construct_model(inputShape):
    inputLayer = keras.layers.Input(inputShape)

    conv1 = keras.layers.Conv1D(filters=64, kernel_size=3, padding="same")(inputLayer)
    conv1 = keras.layers.BatchNormalization()(conv1)
    conv1 = keras.layers.ReLU()(conv1)

    conv2 = keras.layers.Conv1D(filters=64, kernel_size=3, padding="same")(conv1)
    conv2 = keras.layers.BatchNormalization()(conv2)
    conv2 = keras.layers.ReLU()(conv2)

    conv3 = keras.layers.Conv1D(filters=64, kernel_size=3, padding="same")(conv2)
    conv3 = keras.layers.BatchNormalization()(conv3)
    conv3 = keras.layers.ReLU()(conv3)

    gap = keras.layers.GlobalAveragePooling1D()(conv3)

    output_layer = keras.layers.Dense(num_classes, activation="softmax")(gap)

    return keras.models.Model(inputs=inputLayer, outputs=output_layer)
