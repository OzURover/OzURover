import matplotlib.pyplot as plt
import pickle
import numpy as np
import random

pkl_file = open('data.pkl', 'rb')
dataX = pickle.load(pkl_file)
dataY = pickle.load(pkl_file)

sortedX = sorted(dataX)
sortedY = sorted(dataY)

robotX = random.uniform(sortedX[20], sortedX[len(sortedX)-20])
robotY = random.uniform(sortedY[20], sortedY[len(sortedY)-20])

plt.scatter(dataX, dataY, color='b', marker='o', s=1)
plt.scatter(robotX, robotY, color='r', marker='*', s=100)

plt.show()
