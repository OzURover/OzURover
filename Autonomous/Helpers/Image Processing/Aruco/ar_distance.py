
import numpy as np
import math
import cv2
import cv2.aruco as aruco
import glob


class ARDistance:

	mtx, dist = None, None

	markerLength = 20  # cm
	calib_10cm = 0.14142695990350523  # factor

	def __init__(self):
	 	# ---------------------- CALIBRATION ---------------------------
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		objp = np.zeros((6*7, 3), np.float32)
		objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
		square_size = 0.015  # cm
		for i in range(len(objp)):
			objp[i][0] = objp[i][0] * square_size
			objp[i][1] = objp[i][1] * square_size
			objp[i][2] = objp[i][2] * square_size
		objpoints = []
		imgpoints = []
		images = glob.glob('helpers/mapper/calib_images/*.jpg')

		for fname in images:
			img = cv2.imread(fname)
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
			ret, corners = cv2.findChessboardCorners(gray, (7, 6), None)

			if ret == True:
				objpoints.append(objp)

				corners2 = cv2.cornerSubPix(
					gray, corners, (11, 11), (-1, -1), criteria)
				imgpoints.append(corners2)

				img = cv2.drawChessboardCorners(img, (7, 6), corners2, ret)

			ret, self.mtx, self.dist, rvecs, tvecs = cv2.calibrateCamera(
				objpoints, imgpoints, gray.shape[::-1], None, None)

	def get_distance(self, frame):
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
		parameters = aruco.DetectorParameters_create()
		parameters.adaptiveThreshConstant = 10

		corners, ids, rejectedImgPoints = aruco.detectMarkers(
			gray, aruco_dict, parameters=parameters)

		if np.all(ids != None):
			rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
				corners, self.markerLength, self.mtx, self.dist)

			for i in range(0, ids.size):
				aruco.drawAxis(frame, self.mtx, self.dist, rvec[i], tvec[i], 0.1)
			aruco.drawDetectedMarkers(frame, corners)

			x, y = 10000, 10000
			xi, yi = 0, 0
			for i in range(2):
				x = corners[0][0][i][0] if corners[0][0][i][0] < x else x
				y = corners[0][0][i][1] if corners[0][0][i][1] < y else y
				xi = corners[0][0][i][0] if corners[0][0][i][0] > xi else xi
				yi = corners[0][0][i][1] if corners[0][0][i][1] > yi else yi

			strg = ''
			for i in range(0, ids.size):
				strg += str(ids[i][0])+', '

			rotMat, _ = cv2.Rodrigues(rvec)
			angle = self.rotationMatrixToEulerAngles(rotMat)
			distance_factor = self.euclideanDistanceOfTvec(tvec[0][0])

			return (round(distance_factor*31/self.calib_10cm, 1), round(angle[1], 3) * 100)

		return 0, 0

	def isRotationMatrix(self, R):
		Rt = np.transpose(R)
		shouldBeIdentity = np.dot(Rt, R)
		I = np.identity(3, dtype=R.dtype)
		n = np.linalg.norm(I - shouldBeIdentity)
		return n < 1e-6

	def rotationMatrixToEulerAngles(self, R):
		assert(self.isRotationMatrix(R))
		sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
		singular = sy < 1e-6

		if not singular:
			x = math.atan2(R[2, 1], R[2, 2])
			y = math.atan2(-R[2, 0], sy)
			z = math.atan2(R[1, 0], R[0, 0])
		else:
			x = math.atan2(-R[1, 2], R[1, 1])
			y = math.atan2(-R[2, 0], sy)
			z = 0

		return np.array([x, y, z])

	def euclideanDistanceOfTvec(self, tvec):
		return math.sqrt(math.pow(tvec[0], 2) + math.pow(tvec[1], 2) + math.pow(tvec[2], 2))
