#!/usr/bin/python
from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
import math

class QRDistance:
  
  def __init__(self):
    # Calibrate camera
    KNOWN_DISTANCE = 27.0 #cm
    self.KNOWN_WIDTH = 14.8 #cm

    im = cv2.imread('27cm.png')
    decodedObjects = pyzbar.decode(im)
    width = self.calculate_and_draw(im, decodedObjects, is_angle=False)

    self.FOCAL_LENGTH = (width * KNOWN_DISTANCE) / self.KNOWN_WIDTH

  def get_distance(self, im):
    # Read image
    decodedObjects = pyzbar.decode(im)
    if len(decodedObjects) > 0:
      WIDTH, ANGLE = self.calculate_and_draw(im, decodedObjects)
      DISTANCE = self.distance_to_camera(WIDTH)
      return (DISTANCE, ANGLE, im)
    else:
      return ("NA", "NA", im)

  def distance_to_camera(self, perWidth):
    # compute and return the distance from the maker to the camera
    return (self.KNOWN_WIDTH * self.FOCAL_LENGTH) / perWidth

  # Display barcode and QR code location
  def calculate_and_draw(self, im, decodedObjects, is_angle=True):
      width = 0
      angle = 0
      # Loop over all decoded objects
      for decodedObject in decodedObjects:
          points = decodedObject.polygon

          # If the points do not form a quad, find convex hull
          if len(points) > 4:
              hull = cv2.convexHull(np.array([point for point in points],
                                    dtype=np.float32))
              hull = list(map(tuple, np.squeeze(hull)))
          else:
              hull = points
          # Number of points in the convex hull
          n = len(hull)
          # Draw the convext hull and calculate width and angle
          colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (0, 0, 0)]
          vertical_lines = []
          for j in range(0, n):
            f = hull[j]
            s = hull[(j + 1) % n]
            distance = math.sqrt(math.pow(f.x-s.x, 2) + math.pow(f.y-s.y, 2))
            width = distance if distance > width else width
            if j % 2 == 0: vertical_lines.append(distance)
            cv2.line(im, f, s, colors[j], 2)

          if is_angle: angle = math.asin(abs(self.distance_to_camera(vertical_lines[0]) - self.distance_to_camera(vertical_lines[1]))/self.KNOWN_WIDTH)
        
      if is_angle: 
        return (width, angle) 
      else: 
        return width
			