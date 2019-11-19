import numpy as np
from matplotlib.ticker import MaxNLocator
import matplotlib
from helpers.mapper.optimal_path import OptimalPath
import os
import matplotlib.pyplot as plt
import xml.etree.cElementTree as ET
import timeit


def draw(s, e):
	global fig, xm, ym, m

	# Timer
	start = timeit.default_timer()
	r, nr = OP.create_route(s, e)
	stop = timeit.default_timer()

	delta = 1
	x = np.arange(0, xm, delta)
	y = np.arange(0, ym, delta)
	X, Y = np.meshgrid(x, y)
	Z = []

	for yi in range(ym):
		ya = []
		for xi in range(xm):
			ya.append(m.weights[(xi, yi)])
		Z.append(ya)

	Z = np.array(Z)
	# plt.style.use('dark_background')
	levels = MaxNLocator(nbins=50).tick_values(Z.min(), Z.max())
	cmap = plt.get_cmap('RdYlGn_r')

	gs = fig.add_gridspec(4, 3)
	ax0 = fig.add_subplot(gs[:3, :])
	ax0.set_title('Dijkstra Path and Level Map (time: ' +
				  str(round((stop - start)*1000, 2)) + ' ms)')
	ax1 = fig.add_subplot(gs[-1, :])
	ax1.set_title('Route Level Graph')

	cf = ax0.contourf(X, Y, Z, levels=levels, cmap=cmap)
	fig.colorbar(cf, ax=ax0)

	# Path
	l_xs = [x for (x, _) in r]
	l_ys = [y for (_, y) in r]

	ax0.plot(l_xs, l_ys, 'o', color="cyan", markersize=2)

	# Simple Path
	l_xs = [x for (x, _) in nr]
	l_ys = [y for (_, y) in nr]

	ax0.plot(l_xs, l_ys, 'o', color="red",
			 markersize=4, linestyle='--', linewidth=1)

	x = np.arange(0.0, len(r), 1)
	y = []
	for i in range(len(r)):
		y.append(m.weights[(r[i][0], r[i][1])])
	y = np.array(y)

	ax1.plot(x, y)
	ax1.fill_between(x, y, -1)
	ax1.set_ylim(bottom=0)
	plt.yticks(np.arange(min(y), max(y)+0.1, max(y)/3))
	ax1.set_xlim(left=0, right=len(r)-1)


def onclick(event):
	plt.clf()
	draw((0, 0), (int(event.xdata), int(event.ydata)))
	plt.draw()


def press(event):
	global waypoints
	print("press", event.key)
	if event.key.isnumeric():
		i = int(event.key)
		if i >= len(waypoints):
			return False
		plt.clf()
		if i == 0:
			draw((0, 0), waypoints[i])
		else:
			draw(waypoints[i-1], waypoints[i])
		plt.draw()


if __name__ == "__main__":
	global fig, xm, ym, m
	global waypoints
	script_dir = os.path.dirname(__file__)
	file_path = os.path.join(script_dir, 'data/DTM.csv')
	OP = OptimalPath()

	fig = plt.figure(constrained_layout=True)
	fig.canvas.mpl_connect('button_press_event', onclick)
	fig.canvas.mpl_connect('key_press_event', press)
	xm, ym, m = OP.load_map(file_path, bound=3)
	draw((0, 0), (10, 10))

	file_path = os.path.join(script_dir, "data/WaypointsDemo.xml")
	tree = ET.parse(file_path)
	root = tree.getroot()
	waypoints = []

	for child in root:
		waypoint = (child[0].text, child[1].text, child[2].text)
		waypoint = (int(float(waypoint[1]))*2, int(float(waypoint[2]))*2)
		waypoints.append(waypoint)

	plt.show()
