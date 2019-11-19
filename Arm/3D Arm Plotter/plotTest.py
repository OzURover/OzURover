from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.quiver(0, 0, 5, 0, 0, 1, length=5 ,arrow_length_ratio = 0.01)

ax.quiver(5, 5, 5, 0.5, 0.5, 0, length=10 ,arrow_length_ratio = 0.01)


plt.show()
