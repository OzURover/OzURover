"""
Purpose of this file: Fast & Efficient algorithm to find optimum path for given target.

Algorithm goal: Find a path with RRT but given +-5% of the height on the nearest position to target at a given iteration. Also exclude the data behind the dominant axis (perpendicular line to the travlled line for birds-eye view)

Useful Links:
Filter optimization: https://towardsdatascience.com/speeding-up-python-code-fast-filtering-and-slow-loops-8e11a09a9c2f
Fast render?
"""
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import sys
import numpy as np

import noise

## Helper Functions for Terrain Mapping ###
shape = (600, 300)
pts = []
def add_point(x, y, z):
    pts.append([-(shape[0] / 2) + x, -(shape[1] / 2) + y, z])

def plot(shape, z):
    global pts
    vw, vh = shape
    ## Create a GL View widget to display data
    app = QtGui.QApplication([])
    w = gl.GLViewWidget()
    w.show()
    w.setWindowTitle("pyqtgraph example: GLSurfacePlot")
    w.setCameraPosition(distance=50)

    ## Add a grid to the view
    g = gl.GLGridItem()
    g.setSize(vw, vh)
    g.setDepthValue(40)  # draw grid after surfaces since they may be translucent
    w.addItem(g)

    # Plot contour
    p1 = gl.GLSurfacePlotItem(z=z, shader="shaded", color=(0.5, 0.5, 1, 1))
    p1.scale(1.0, 1.0, 1.0)
    p1.translate(-vw / 2, -vh / 2, 0)
    w.addItem(p1)

    # Plot Line
    pts = np.array(pts)
    plt = gl.GLLinePlotItem(pos=pts, color=(0, 1, 0, 1), antialias=True, width=5.0)
    w.addItem(plt)

    ## Start Qt event loop unless running in interactive mode.
    if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
        QtGui.QApplication.instance().exec_()


## Create Random Terrain ##
scale = 100
octaves = 4
persistence = 0.2
lacunarity = 3.0
seed = 100#np.random.randint(0, 100)

world = np.zeros(shape)
for i in range(shape[0]):
    for j in range(shape[1]):
        world[i][j] = noise.pnoise2(
            i / scale,
            j / scale,
            octaves=octaves,
            persistence=persistence,
            lacunarity=lacunarity,
            repeatx=600,
            repeaty=300,
            base=seed,
        )
world *= 40

## Optimum Path Helpers ##

# Filter map to get desired height slice
def hslice(z, rate=0.05):
    if z < 0:
        cond = (world < (z*(1+rate))) | (world > (z*(1-rate)))
    else:
        cond = (world < (z*(1-rate))) | (world > (z*(1+rate)))
    return np.where(cond, -100, world)



## Find the Optimum Path ##

# Set start and goal
startx, starty = 50, 150
endx, endy = 550, 150

add_point(startx, starty, world[startx, starty])
add_point(startx, starty+1, world[startx, starty])

## APPLY RRT and change to cost function to account for slope with tan?


## Plot the map and the optimum path ##
plot(shape, hslice(world[startx, starty], rate=.3))
