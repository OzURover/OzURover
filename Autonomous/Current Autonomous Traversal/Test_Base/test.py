import numpy as np
import math
import matplotlib.pyplot as plt

# Define the known points
x = [0, 1]
y = [0, -1]

relative_x = x[1] - x[0]
relative_y = y[1] - y[0]

tan = relative_y * 1.0 /relative_x

print("tan:", tan)

if relative_y >= 0:
    if relative_x >= 0:
        line_angle = abs(math.degrees(np.arctan(tan)))
    else:
        line_angle = 180 - abs(math.degrees(np.arctan(tan)))
else:
    if relative_x >= 0:
        line_angle = 360 - abs(math.degrees(np.arctan(tan)))
    else:
        line_angle = 180 + abs(math.degrees(np.arctan(tan)))

print("Angle:", line_angle)
