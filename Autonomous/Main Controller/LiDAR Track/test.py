import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.gridspec as gridspec
import numpy as np
import math
from algorithms import rdp
import timeit
import sys
import pickle
import os


def rotate_left(array, shift):
	length = len(array)
	overflow = length * (shift//length + 1)
	return [array[i+shift - overflow] for i in range(length)]


epsilon = 8
iteration = 0
total = 0
real = 0
move = 1
data = []
error = 0



def update(frame):
	global axes, epsilon, start, iteration, total, move, real, obstacle, error, distance_to_ground

	if real > 100:
		data.append({
			"real": real,
			"calculated": total,
			"error": error,
			"obstacle": obstacle,
			"epsilon": epsilon,
			"distance_to_ground": distance_to_ground
		})
		obstacle = obstacle + 1
		total = 0
		real = 0
		epsilon = 8

	if obstacle > distance_to_ground * 0.70:
		script_dir = os.path.dirname(__file__)
		file_path = os.path.join(script_dir, 'results.csv')
		outfile = open(file_path, 'w')
		for i, item in enumerate(data):
			if i == 0:
				for k, _ in item.items():
					outfile.write(str(k) + ', ')
				outfile.write("\n")
			for _, v in item.items():
				outfile.write(str(v) + ', ')
			outfile.write("\n")
		outfile.close()
		exit(0)

	points = []
	referances = None
	zero = None
	default = np.random.uniform(low=0, high=(distance_to_ground*0.2), size=(360,))
	default = [x if x > distance_to_ground*0.12 else distance_to_ground*0.12 for x in default]
	default[170:190] = [obstacle for _ in range(20)]
	l_ys_r = default
	real = real + math.sqrt(distance_to_ground**2 + distance_to_ground**2 - 2*distance_to_ground*distance_to_ground *
							math.cos(math.radians(move*2 * 0.25)))

	for i in range(6):
		if i % 2 != 0:
			continue
		l_xs = list(range(360))
		l_ys_r = rotate_left(l_ys_r, -move)
		l_ys = l_ys_r

		l_ys = [distance_to_ground-y for y in l_ys]

		if len(axes[i].lines) > 1:
			axes[i].lines = []
		axes[i].plot(l_xs, l_ys, 'o', color="blue", markersize=2)

		l_ys = [(i, y) for i, y in enumerate(l_ys)]
		l_ys, mask = rdp(l_ys, epsilon=epsilon)
		l_ys = l_ys.tolist()
		indices = [i for i, x in enumerate(mask) if x == True]
		l_ys_m = np.zeros(360)
		ii = 0
		for d in range(360):
			if d in indices:
				l_ys_m[d] = l_ys[ii][1]
				ii = ii + 1
			else:
				l_ys_m[d] = -5

		if i == 0:
			points = []
			referances = [(i, l_ys_m[i]) for i, x in enumerate(
				mask) if x == True][1:-1] if len(indices) > 2 else None
			zero = indices
			zero = zero[1:-1]
		else:
			if len(l_ys) > 2:
				points.append(l_ys)

		if len(indices) < 6:
			epsilon = epsilon - 0.5
		elif len(indices) > 8:
			epsilon = epsilon + 0.1

		if zero is not None:
			for xc in zero:
				axes[i].axvline(x=xc, color="red", linestyle='--', linewidth=1)

		axes[i].plot(l_xs, l_ys_m, 'o', color="red", markersize=3)

	if referances is not None and len(points) > 1:
		ERR_MARGIN = 0.5
		ERR_INDEX = 15
		out = ""
		diffs = []

		# print(referances)
		# print(points[0])
		# print(points[1])

		for point in referances:
			try:
				a = list(filter(lambda x: abs(
					x[1] - point[1]) < ERR_MARGIN and abs(x[0] - point[0]) < ERR_INDEX and x[0] > point[0], points[0]))[0]
				b = list(filter(lambda x: abs(
					x[1] - point[1]) < ERR_MARGIN and abs(x[0] - point[0]) < ERR_INDEX and x[0] > point[0], points[1]))[0]
			except Exception:
				continue
			out = out + "\nPoint: " + \
				str(point) + " was found on: " + str(a) + " and " + str(b)
			indexes = [a[0], b[0]]
			dists = [a[1], b[1]]
			diffs.append(max(indexes) - point[0])

		if out != "":
			stop = timeit.default_timer()
			duration = round(stop - start, 2)
			#print(str(duration) + " seconds\n")
			start = stop
			#print(out)
			#print(diffs)
			#print("Iter: " + str(np.average(diffs)) + " iter")
			#print("Movement: " + str(np.average(diffs) * 0.25) + " degree")
			distance = math.sqrt(dists[0]**2 + dists[1]**2 - 2*dists[0]
							* dists[1]*math.cos(math.radians(np.average(diffs) * 0.25)))
			#print("Actual Movement: " + str(round(distance, 3)) + " cm")
			error = (1-(min([total, real])/max([total, real])))*100.0
			title = "Actual Movement: " + str(round(total, 2)) + " cm (in " + str(duration) + " seconds) ["+str(iteration)+"/"+str(obstacle)+"][real: " + str(
				round(real, 2)) + " cm][error: " + str(round(error, 2)) + "%]"
			axes[0].set_title(title + " Epsilon: " + str(round(epsilon, 2)))
			total = total + distance
			iteration = iteration + 1


if __name__ == "__main__":
	global axes, start, distance_to_ground, obstacle
	distance_to_ground = 30
	obstacle = distance_to_ground * 0.275
	start = timeit.default_timer()
	axes = []
	# Setup Axes
	fig = plt.figure(figsize=(6, 6))

	gs1 = gridspec.GridSpec(6, 1)
	gs1.update(hspace=0.0)

	for i in range(5):
		# i = i + 1 # grid spec indexes from 0
		ax = plt.subplot(gs1[i])
		axes.append(ax)
		plt.rcParams["font.size"] = "7"
		plt.axis('on')
		ax.set_ylim([0, distance_to_ground])

		l_xs = list(range(360))
		l_ys = np.zeros(360)  # np.random.uniform(low=0, high=60, size=(360,))

		ax.plot(l_xs, l_ys, 'o', color="blue",
					markersize=2)

	animation = FuncAnimation(fig, update, interval=1)
	plt.show()
