#!/usr/bin/env python
import rospy
import time
import sys
import timeit
import message_filters
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Point

constants = {
	"LM": 0.482372,
	"LB": 1.733848,
	"RM": 0.479325,
	"RB": 0.464739,
	"CR": 50.265482
}  # x * CR/LM

DATAS = {
	"RM": {
		"fault": 0,
		"lock": 0},
	"RB": {
		"fault": 0,
		"lock": 0},
	"LM": {
		"fault": 0,
		"lock": 0},
	"LB": {
		"fault": 0,
		"lock": 0}
}

BNOS = {
	"BNO": {
		"fault": 0,
		"lock": 0},
	"BNO2": {
		"fault": 0,
		"lock": 0}
}

BNOS_T = {
	"BNO": {
		"secs": 0,
		"secs_2": 0,
		"tick": 0},
	"BNO2": {
		"secs": 0,
		"secs_2": 0,
		"tick": 0},
}


def bno(data):
	global multi_bno
	BNOS["BNO"] = {
		"header": data.header,
		"point": data.point,
		"fault": BNOS["BNO"]["fault"],
		"lock": BNOS["BNO"]["lock"]
	}
	if not multi_bno:
		BNOS["BNO2"] = {
			"header": data.header,
			"point": data.point,
			"fault": BNOS["BNO2"]["fault"],
			"lock": BNOS["BNO2"]["lock"]
		}


def bno2(data):
	BNOS["BNO2"] = {
		"header": data.header,
		"point": data.point,
		"fault": BNOS["BNO2"]["fault"],
		"lock": BNOS["BNO2"]["lock"]
	}


def rm(data):
	global TICK
	p = Point()
	p.x = data.point.x * (constants["CR"] / constants["RM"])
	p.y = data.point.y
	p.z = 0
	DATAS["RM"] = {
		"header": data.header,
		"point": p,
		"tick": TICK,
		"fault": DATAS["RM"]["fault"],
		"lock": DATAS["RM"]["lock"]
	}


def rb(data):
	global TICK
	p = Point()
	p.x = data.point.x * (constants["CR"] / constants["RB"])
	p.y = data.point.y
	p.z = 0
	DATAS["RB"] = {
		"header": data.header,
		"point": p,
		"tick": TICK,
		"fault": DATAS["RB"]["fault"],
		"lock": DATAS["RB"]["lock"]
	}


def lm(data):
	global TICK
	p = Point()
	p.x = data.point.x * (constants["CR"] / constants["LM"])
	p.y = data.point.y
	p.z = 0
	DATAS["LM"] = {
		"header": data.header,
		"point": p,
		"tick": TICK,
		"fault": DATAS["LM"]["fault"],
		"lock": DATAS["LM"]["lock"]
	}


def lb(data):
	global TICK
	p = Point()
	p.x = data.point.x * (constants["CR"] / constants["LB"])
	p.y = data.point.y
	p.z = 0
	DATAS["LB"] = {
		"header": data.header,
		"point": p,
		"tick": TICK,
		"fault": DATAS["LB"]["fault"],
		"lock": DATAS["LB"]["lock"]
	}


if __name__ == "__main__":
	global TICK, multi_bno
	TICK = 0
	rospy.init_node("redunancy_controller")

	# Gyro subscribers
	multi_bno = sys.argv[1] == "true"
	if multi_bno:
		bno_subscriber = rospy.Subscriber("bno_gyro", PointStamped, bno)
		bno_subscriber_2 = rospy.Subscriber("bno_gyro_2", PointStamped, bno2)
	else:
		bno_subscriber = rospy.Subscriber("bno_gyro_2", PointStamped, bno)

	# Subscriber for wheel encoders
	right_mid_encoder_sub = rospy.Subscriber("RM_info", PointStamped, rm)
	left_mid_encoder_sub = rospy.Subscriber("LM_info", PointStamped, lm)
	right_back_encoder_sub = rospy.Subscriber("RB_info", PointStamped, rb)
	left_back_encoder_sub = rospy.Subscriber("LB_info", PointStamped, lb)

	# Synchronized publishers
	BNO_S = rospy.Publisher("bno_gyro/s", PointStamped, queue_size=5)
	RM_S = rospy.Publisher("RM_info/s", PointStamped, queue_size=5)
	RB_S = rospy.Publisher("RB_info/s", PointStamped, queue_size=5)
	LM_S = rospy.Publisher("LM_info/s", PointStamped, queue_size=5)
	LB_S = rospy.Publisher("LB_info/s", PointStamped, queue_size=5)

	try:
		print("REDUNANCY CONTROLLER IS STARTING...")
		start = timeit.default_timer()
		rospy.rostime.wallsleep(0.1)
		error_count = 0
		bno_error_count = 0
		fatal_count = 0
		bno_fatal_count = 0
		reset = False
		err_time = 30
		bno_last_seq = 0
		bno_last_seq_2 = 0

		#debug
		s_count = 0
		while not rospy.core.is_shutdown():
			if len(BNOS) > 0:
				bTick = BNOS_T["BNO"]["secs"]
				b2Tick = BNOS_T["BNO2"]["secs"]

				bTick_2 = BNOS_T["BNO"]["secs_2"]
				b2Tick_2 = BNOS_T["BNO2"]["secs_2"]

				if bTick == bTick_2:
					BNOS_T["BNO"]["tick"] = BNOS_T["BNO"]["tick"] + 1
				else:
					BNOS_T["BNO"]["tick"] = 0

				if b2Tick == b2Tick_2:
					BNOS_T["BNO2"]["tick"] = BNOS_T["BNO2"]["tick"] + 1
				else:
					BNOS_T["BNO2"]["tick"] = 0

				if BNOS["BNO"].has_key("point") != 1:
					continue

				bNow = BNOS["BNO"]["point"]
				bStatus = BNOS["BNO"]["point"].z == 5
				bLive = BNOS_T["BNO"]["tick"] < 15
				b2Now = BNOS["BNO2"]["point"]
				b2Status = BNOS["BNO2"]["point"].z == 5
				b2Live = BNOS_T["BNO2"]["tick"] < 15

				if (bStatus and b2Status) and (bLive and b2Live):
					if abs(bNow.x - b2Now.x) < max([bNow.x, b2Now.x]) * 0.8:
						BNO_S.publish(BNOS["BNO"]["header"], bNow)
					else:
						BNO_S.publish(BNOS["BNO2"]["header"], b2Now)

					if BNOS["BNO"]["lock"]:
						print("BNO restored")
						bno_error_count = bno_error_count - 1 if bno_error_count > 0 else 0
						BNOS["BNO"]["lock"] = 0

					if BNOS["BNO2"]["lock"]:
						print("BNO2 restored")
						bno_error_count = bno_error_count - 1 if bno_error_count > 0 else 0
						BNOS["BNO2"]["lock"] = 0

					bno_fatal_count = 0
				else:
					d = None
					if bStatus and bLive:
						BNO_S.publish(BNOS["BNO"]["header"], bNow)
						d = "BNO2"
						bno_fatal_count = 0
					elif b2Status and b2Live:
						BNO_S.publish(BNOS["BNO2"]["header"], b2Now)
						d = "BNO"
						bno_fatal_count = 0
					else:
						if bno_fatal_count < 3*err_time and bno_fatal_count % err_time == 0 and TICK > 6:
							print("WARNING: GYRO SYSTEM COMPRIMISED. TRYING AGAIN")
						if bno_fatal_count > 3*err_time:
							print("FATAL ERROR: NO GYRO FOUND")
							exit(2)
						bno_fatal_count = bno_fatal_count + 1
						# Publishing empty message
						h = Header()
						h.stamp = rospy.Time.now()
						BNO_S.publish(h, bNow)

					if d != None:
						if not BNOS[d]["lock"]:
							BNOS[d]["lock"] = 1
							bno_error_count = bno_error_count + 1
							print(d + " gyro failed and swapped with backup")

			if len(DATAS) > 0:
				for d in DATAS:
					c = DATAS.get(d)
					cTick = -1
					if c.has_key("tick") == 1:
						cTick = DATAS[d]["tick"]
					if cTick == TICK:
						header = c["header"]
						point = c["point"]
						if d == "RM":
							RM_S.publish(header, point)
						elif d == "RB":
							RB_S.publish(header, point)
						elif d == "LM":
							LM_S.publish(header, point)
						elif d == "LB":
							LB_S.publish(header, point)
					else:
						# Get any active encoder
						active = None
						sub = None
						for dd in DATAS:
							c = DATAS.get(dd)
							if c.has_key("tick") == 1:
								cTick = DATAS[dd]["tick"]
								if cTick == TICK:
									active = c
									sub = dd
									break

						if active is None:
							if fatal_count < 3*err_time and fatal_count % err_time == 0:
								print(
									"WARNING: NO SUBSTITUTE ENCODER FOUND. TRYING AGAIN")
							if fatal_count > 3*err_time:
								print("FATAL ERROR: NO SUBSTITUTE ENCODER FOUND")
								exit(2)
							fatal_count = fatal_count + 1
							error_count = error_count + 1
							# Publishing empty message
							header = Header()
							header.stamp = rospy.Time.now()
							point = Point()
						else:
							header = active["header"]
							point = active["point"]

						# Trip fail on faulty encoder
						if not DATAS[d]["lock"]:
							DATAS[d]["fault"] = 1
							DATAS[d]["lock"] = 1
							error_count = error_count + 1

							print(d + " encoder failed but " + str(sub) + " was put in to play")
						else:
							DATAS[d]["fault"] = 0

						if error_count + bno_error_count > 0 and TICK > 0 and TICK % err_time == 0:
							if error_count > 0:
								point.z = 1
							reset = True
						else:
							point.z = DATAS[d]["fault"]

						if d == "RM":
							RM_S.publish(header, point)
						elif d == "RB":
							RB_S.publish(header, point)
						elif d == "LM":
							LM_S.publish(header, point)
						elif d == "LB":
							LB_S.publish(header, point)

			TICK = TICK + 1
			if reset:
				print("=======RESET " + str(err_time/10) + " sec period =======")
				error_count = 0
				TICK = 0
				reset = False
				for e in DATAS:
					DATAS[e]["lock"] = 0
				for e in BNOS:
					BNOS[e]["lock"] = 0

			BNOS_T["BNO"]["secs"] = BNOS["BNO"]["header"].stamp.secs
			BNOS_T["BNO2"]["secs"] = BNOS["BNO2"]["header"].stamp.secs
			rospy.rostime.wallsleep(0.1)
			BNOS_T["BNO"]["secs_2"] = BNOS["BNO"]["header"].stamp.secs
			BNOS_T["BNO2"]["secs_2"] = BNOS["BNO2"]["header"].stamp.secs

			now = timeit.default_timer()
			if now - start > 30:
				start = now
				print("RC STILL ACTIVE")
	except KeyboardInterrupt:
		rospy.core.signal_shutdown('keyboard interrupt')
