#!/usr/bin/env python
import dynamixel
import time
class ServoController:

	def __init__(self):
		self.hprev = None
		self.vprev = None

	def start(self):
		serial = dynamixel.SerialStream(port='/dev/ttyUSB0', baudrate=1000000, timeout=1)
		self.net = dynamixel.DynamixelNetwork(serial)
		self.net.scan(1, 5)
		dxl_count = 0
		self.motors = [None]*5

		print("Searching Dynamixel motors...")
		
		for dyn in self.net.get_dynamixels():
			self.motors[dyn.id-1] = dyn
			dxl_count = dxl_count+1

		print("Found " + str(dxl_count) + " Dynamixel motors. Those are:")
		for d in self.motors:
			if(d != None):
				print("ID: "+str(d.id)+" Found")

	def get_pos(self):
		v = (self.motors[3].current_position - 1024/2) / (1024/300.0)
		h = (self.motors[4].current_position - 1024/2) / (1024/300.0)
		return h, v

	def cam_ver(self, deg=0, inc=0, speed=0.7):
		if inc != 0:
			while True:
				pos = self.motors[3].current_position
				if self.vprev is None:
					self.vprev = pos
					self.pv = 1
				elif self.vprev == pos:
					self.pv = self.pv + 1
				else:
					self.vprev = pos
					self.pv = 1
					break

				d = pos + (inc * self.pv)
				sh, _ = self.get_pos()
				if sh >= 0 or sh < -90:
					if (d >= 400 and d <= 665):
						self.motors[3].moving_speed = int(1023 * speed)
						self.motors[3].torque_limit = 1023
						self.motors[3].max_torque = 1023
						self.motors[3].goal_position = int(d)
						self.net.synchronize()
					else:
						print("Error [Overload]")
						return False
				elif sh < 0 and sh >= -90:
					if (d >= 400 and d <= 603):
						self.motors[3].moving_speed = int(1023 * speed)
						self.motors[3].torque_limit = 1023
						self.motors[3].max_torque = 1023
						self.motors[3].goal_position = int(d)
						self.net.synchronize()
					else:
						print("Error [Overload]")
						return False
				else:
					print("Error [Overload]")
					return False

				time.sleep(0.1)
		else:
			d = 1024/2 + deg*(1024/300.0)
			d = 1023 if d == 1024 else d
			sh, _ = self.get_pos()
			if sh >= 0 or sh < -90:
				if (d >= 400 and d <= 665):
					self.motors[3].moving_speed = int(1023 * speed)
					self.motors[3].torque_limit = 1023
					self.motors[3].max_torque = 1023
					self.motors[3].goal_position = int(d)
					self.net.synchronize()
				else:
					print("Error [Overload]")
					return False
			elif sh < 0 and sh >= -90:
				if (d >= 400 and d <= 603):
					self.motors[3].moving_speed = int(1023 * speed)
					self.motors[3].torque_limit = 1023
					self.motors[3].max_torque = 1023
					self.motors[3].goal_position = int(d)
					self.net.synchronize()
				else:
					print("Error [Overload]")
					return False
			else:
				print("Error [Overload]")
				return False
		return True

	def cam_hor(self, deg=0, inc=0, speed=0.7):
		if inc != 0:
			while True:
				pos = self.motors[4].current_position
				if self.hprev is None:
					self.hprev = pos
					self.ph = 1
				elif self.hprev == pos:
					self.ph = self.ph + 1
				else:
					self.hprev = pos
					self.ph = 1
					break

				d = pos + (inc * self.ph)
				d = 1023 if d == 1024 else d
				if(d <= 1023):
					self.motors[4].moving_speed = int(1023 * speed)
					self.motors[4].torque_limit = 1023
					self.motors[4].max_torque = 1023
					self.motors[4].goal_position = int(d)
					self.net.synchronize()

				time.sleep(0.1)
		else:
			d = 1024/2 + deg*(1024/300.0)
			d = 1023 if d == 1024 else d
			if(d <= 1023):
				self.motors[4].moving_speed = int(1023 * speed)
				self.motors[4].torque_limit = 1023
				self.motors[4].max_torque = 1023
				self.motors[4].goal_position = int(d)
				self.net.synchronize()
