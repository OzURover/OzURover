import os
import math
import random
import matplotlib.pyplot as plt
import xml.etree.cElementTree as ET
from helpers.mapper.algorithms import calculate_route, calculate_position

if __name__ == "__main__":
	markers = [(0, 104.40, -106.7), (1, 40.0, 90.0), (2, 170.0, 0.0)] # id, dist, deg
	landmarks = [] # id, x, y
	waypoints = []

	script_dir = os.path.dirname(__file__)
	file_path = os.path.join(script_dir, "data/Landmarks.xml")
	tree = ET.parse(file_path)
	root = tree.getroot()

	for child in root:
		landmark = (child[0].text, child[1].text, child[2].text)
		landmark = tuple(float(x) for x in landmark)
		landmarks.append(landmark)

	file_path = os.path.join(script_dir, "data/Waypoints.xml")
	tree = ET.parse(file_path)
	root = tree.getroot()

	for child in root:
		waypoint = (child[0].text, child[1].text, child[2].text)
		waypoint = tuple(float(x) for x in waypoint)
		waypoints.append(waypoint)

	# Setup Axes
	#plt.style.use('dark_background')
	fig = plt.figure()
	ax = fig.add_subplot(1, 1, 1)
	#plt.xlim(-10, 10)
	#plt.ylim(-10, 10)
	plt.rcParams["font.size"] = "7"
	ax.set_aspect('equal')
	ax.grid(True, which='both')

	# set the x-spine (see below for more info on `set_position`)
	ax.spines['left'].set_position('zero')

	# turn off the right spine/ticks
	ax.spines['right'].set_color('none')
	ax.yaxis.tick_left()

	# set the y-spine
	ax.spines['bottom'].set_position('zero')

	# turn off the top spine/ticks
	ax.spines['top'].set_color('none')
	ax.xaxis.tick_bottom()

	# Landmarks
	l_xs = [x for (_, x, _) in landmarks]
	l_ys = [y for (_, _, y) in landmarks]

	for i in range(len(landmarks)):
		l = landmarks[i]
		plt.annotate("ID: " + str(int(l[0])),
					xy=(l[1], l[2]), xycoords='data',
					xytext=(+10, +10), textcoords='offset points', fontsize=8,
					arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

	plt.plot(l_xs, l_ys, 'o', color="red", markersize=4)

	# Waypoints
	w_xs = [x for (_, x, _) in waypoints]
	w_ys = [y for (_, _, y) in waypoints]

	for i in range(len(waypoints)):
		w = waypoints[i]
		plt.annotate("ID, X, Y: " + str(int(w[0])) + ", " + str(int(w[1])) + ", " + str(int(w[2])),
					xy=(w[1], w[2]), xycoords='data',
					xytext=(-10, +10), textcoords='offset points', fontsize=8,
					arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=.2"))

	plt.plot(w_xs, w_ys, 'o', color="blue", markersize=4)

	x, y, ref = calculate_position(landmarks, markers, (0, 0))
	if x != -1:
		plt.plot([x], [y], 'o', color="cyan", markersize=10)
	else:
		exit(3)

	colors = ["magenta", "red", "black", "cyan"]
	random.shuffle(colors)
	for i in range(len(waypoints)):
		D, A = calculate_route(ref, waypoints[i], (x, y), landmarks)
		print(i, D, A)

		A = math.radians(A)
		Dx = D * math.sin(A) + x
		Dy = D * math.cos(A) + y
		x2 = [x, Dx]
		ys = [y, Dy]
		label = 'Waypoint ' + str(i)
		ax.plot(x2, ys, 'black', linestyle='--', marker='', color=colors[i], label=label)
	
	ax.legend()
	plt.show()
