import rospy
import yaml
import math
import sys
import argparse
import traceback
import numpy as np
from timeit import default_timer as dt

# Import helpers
from helpers.driver import Driver, DriverStatus
from helpers.lrs import LRS

# Import types
from nav_msgs.msg import Odometry

flag1 = False
flag2 = False

def get_next(robot, markers, scale=2, tol=0.8):
    global flag1, flag2

    C = [(markers[0][0] + markers[1][0]) / 2, (markers[0][1] + markers[1][1]) / 2]
    s = math.sin(math.radians(90))
    c = math.cos(math.radians(90))

    points90 = []
    for (mx, my) in markers:
        mx -= C[0]
        my -= C[1]

        mx *= scale
        my *= scale

        xnew = mx * c - my * s
        ynew = mx * s + my * c

        print(xnew)

        mx = xnew + C[0]
        my = ynew + C[1]

        points90.append([mx, my])

    points90 = sorted(
        points90,
        key=lambda x: math.sqrt((robot[0] - x[0]) ** 2 + (robot[1] - x[1]) ** 2),
    )

    d_entry = math.sqrt(
        (robot[0] - points90[0][0]) ** 2 + (robot[1] - points90[0][1]) ** 2
    )
    d_c = math.sqrt((robot[0] - C[0]) ** 2 + (robot[1] - C[1]) ** 2)

    if d_c < tol * 0.5 or flag2:
        flag2 = True
        return points90[-1]

    if d_entry < tol or flag1:
        flag1 = True
        return C
    else:
        return points90[0]


def route(payload):
    Driver_Instance.update_pose(payload.pose)


def main(args):
    global Driver_Instance

    rospy.init_node("autonomous", anonymous=True)

    config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)
    Driver_Instance = Driver(gainL=args.GAIN_L, gainA=args.GAIN_A)
    LRS_Instance = LRS(Driver_Instance, clean=args.fresh)

    # Subscribe to relevant topics and services
    rospy.Subscriber(config["odometry_topic"], Odometry, route)

    # Wait for elevation map to be available
    rospy.loginfo("Waiting for topics to become online...")
    rospy.rostime.wallsleep(0.1)
    rospy.wait_for_message(config["odometry_topic"], Odometry, timeout=10)
    rospy.loginfo("OK!")

    try:
        # ROS Loop
        rospy.loginfo("Running the control loop")
        while not rospy.core.is_shutdown():

            if Driver_Instance.flag == DriverStatus.HALT:
                try:
                    ans = input("Execute? [y/n]")
                    if ans == "n":
                        exit(0)
                except ValueError:
                    print("Invalid input")
                    continue
                Driver_Instance.flag = DriverStatus.NORMAL

            start = dt()
            px, py, _ = Driver_Instance.get_current_loc()

            if LRS_Instance.has_landmarks():
                AR = LRS_Instance.dictionary
                valid = 0
                markers = []

                for m_id, m in AR.items():
                    if m_id == 0 or m_id == 1:
                        valid += 1
                        markers.append(m.loc())

                if valid != 2:
                    continue

                x, y = get_next((px, py), markers, scale=2, tol=0.8)

                rospy.logwarn(
                    "\nx={:.2f} :: y={:.2f} :: in {:.2f} ms\n".format(
                        x, y, 1000 * (dt() - start)
                    )
                )
                Driver_Instance.go_to_position(x, y)

            rospy.rostime.wallsleep(0.1)
    except (Exception, rospy.ROSException, KeyboardInterrupt):
        exc_type, exc_value, exc_traceback = sys.exc_info()
        rospy.logfatal("Program crashed or halted")
        traceback.print_exception(
            exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout
        )
        rospy.core.signal_shutdown("exited")
        exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ASD Driver")
    parser.add_argument("--GAIN_A", type=float, default=6.0)
    parser.add_argument("--GAIN_L", type=float, default=50.0)
    parser.add_argument(
        "--fresh", help="Do not use previous backup files", action="store_true"
    )
    main(parser.parse_args())
