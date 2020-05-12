"""
Purpose of this file: Fast & Efficient algorithm to find optimum path for given target.

7/05/2020: Algorithm goal: Find a path with RRT but given +-5% of the height on the nearest position to target at a given iteration. Also exclude the data behind the dominant axis (perpendicular line to the travlled line for birds-eye view)

7/05/2020: Maybe... A modified version of RRT* could be used to skip reloading new height slice. That version could limit itself to use the slice it needs.

8/05/2020: Or... A* with Fuel and Altitude change costmap

10/05/2020: Searching for efficient A*, also looks like it's easy to read HGT file

11/05/2020: Tobler's Hiking Problem is what we need. Apply it! 
Just need to figure out how to generte raster from matrix with given coordiantes
NumPy to R Matrix: https://stackoverflow.com/a/27303951/2289242
Load library and call function: https://rpy2.github.io/doc/v3.0.x/html/robjects_functions.html#callable

11/05/2020: Okay so, if SRTM data is sqare and adjacent cells are sperated by just 1 degree then it's safe to assume (lat, long) -> (X, Y)[that one library which converted to cartesian or just degree/3601]. Then we can just give a matrix with origin and destin (in their respective formats) and generate the path. Then append that route to RTAB-MAP and we can use the route. This also apply for TIF data. Extract and set origin manually via an ARTag

Useful Links:
A* implementation: https://github.com/jrialland/python-astar
Filter optimization: https://towardsdatascience.com/speeding-up-python-code-fast-filtering-and-slow-loops-8e11a09a9c2f
Fast render?
"""
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import sys
import os
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
seed = 100  # np.random.randint(0, 100)

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
        cond = (world < (z * (1 + rate))) | (world > (z * (1 - rate)))
    else:
        cond = (world < (z * (1 - rate))) | (world > (z * (1 + rate)))
    return np.where(cond, -100, world)


## Find the Optimum Path ##

# Set start and goal
startx, starty = 50, 150
endx, endy = 550, 150

add_point(startx, starty, world[startx, starty])
add_point(startx, starty + 1, world[startx, starty])

## APPLY RRT and change to cost function to account for slope with tan?

script_dir = os.path.dirname(__file__)
file_path = os.path.join(script_dir, "N38W111_Utah.hgt")

SAMPLES = 3601  # Change this to 3601 for SRTM1
with open(file_path, "rb") as hgt_data:
    data = (
        np.fromfile(hgt_data, np.dtype(">i2"), SAMPLES * SAMPLES)
        .reshape((SAMPLES, SAMPLES))
        .T
    )

from matplotlib import pyplot as plt
import timeit
import math

data = np.flip(data, axis=1)


from astar import AStar


class MapSolver(AStar):
    def __init__(self, map):
        self.data = map
        self.width = self.height = 3601

    def heuristic_cost_estimate(self, n1, n2):
        (x1, y1) = n1
        (x2, y2) = n2
        return math.hypot(x2 - x1, y2 - y1)

    def gh(self, n):
        return self.data[n]

    def distance_between(self, n1, n2, i):
        hd = self.gh(n2.data) - self.gh(n1.data)
        dist = self.heuristic_cost_estimate(n1.data, n2.data)
        
        if dist > 2.0:
            a = math.atan(hd / math.sqrt(5))
        elif dist > 1.0:
            a = math.atan(hd / math.sqrt(2))
        else:
            a = math.atan(hd)

        return dist * a + hd * a

    def neighbors(self, node):
        x, y = node
        n = [
            (x - 1, y + 1),
            (x, y + 1),
            (x + 1, y + 1),
            (x + 1, y),
            (x + 1, y - 1),
            (x, y - 1),
            (x - 1, y - 1),
            (x - 1, y),
            (x - 1, y + 2),
            (x + 1, y + 2),
            (x + 2, y + 1),
            (x + 2, y - 1),
            (x + 1, y - 2),
            (x - 1, y - 2),
            (x - 2, y - 1),
            (x - 2, y + 1),
        ]
        return [
            (nx, ny) for nx, ny in n if 0 <= nx < self.width and 0 <= ny < self.height
        ]


"""
Still goes up
does not take into account of the previous costs wtf how?
"""

start = timeit.default_timer()
path = MapSolver(data).astar((600, 300), (300, 1000))
end = timeit.default_timer()


path = np.array(list(path))

print(end - start, "secs")

start = timeit.default_timer()

plt.style.use("dark_background")
plt.plot(path[:, 0], path[:, 1], "r", lw=2)
plt.contourf(data, cmap="terrain", levels=list(range(0, 3570, 50)))

end = timeit.default_timer()

plt.colorbar()
plt.show()


print(end - start, "secs")


## Plot the map and the optimum path ##
# plot(shape, hslice(world[startx, starty], rate=.3))
