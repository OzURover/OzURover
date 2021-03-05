import sys
import traceback
import rospy
import math
import yaml
import numpy as np
from threading import Thread
from enum import Enum

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Vector3, Twist
from std_msgs.msg import Header


class DriverStatus(Enum):
    HALT = 0
    NORMAL = 1
    REJECTED = 2
    ACCEPTED = 3
    PREEMPTING = 4
    CANCELLED = 5


class Driver:
    def __init__(self, gainL=5.0, gainA=2.0):
        self.config = yaml.load(open("./config/config.yaml"), Loader=yaml.FullLoader)
        self.path_pub = rospy.Publisher("/asd_core/path", Path, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(self.config["cmd_vel"], Twist, queue_size=10)

        self.GAIN_L = gainL
        self.GAIN_A = gainA

        self.target_goal = (0, 0, 0)
        self.target, self.source, self.latch = (None, None), (None, None), True
        self.flag = DriverStatus.HALT

        self.worker = Thread(name="driver_worker", target=self.daemon_worker)
        self.worker.setDaemon(True)
        self.worker.start()

    def daemon_worker(self):
        try:
            cmd_pub = rospy.Publisher(self.config["cmd_vel"], Twist, queue_size=10)
            rospy.loginfo("Driver Daemon online")

            while not rospy.core.is_shutdown():
                try:
                    if self.flag == DriverStatus.PREEMPTING:
                        cx, cy, cr = self.get_current_loc()
                        tx, ty, _ = self.target_goal

                        delta_angle = math.atan2(ty - cy, tx - cx)
                        delta_distance = math.sqrt((ty - cy) ** 2 + (tx - cx) ** 2)

                        ANG_vel = delta_angle - cr
                        LIN_vel = delta_distance * self.GAIN_L

                        # * Fix ANG_vel
                        if ANG_vel > math.pi:
                            ANG_vel -= 2 * math.pi
                        elif ANG_vel < -math.pi:
                            ANG_vel += 2 * math.pi

                        ANG_vel *= self.GAIN_A

                        rospy.logwarn_throttle(
                            0.2,
                            "Da={:.3f} :: Dd={:.3f} :: Va={:.3f} :: Vl={:.3f}".format(
                                delta_angle, delta_distance, ANG_vel, LIN_vel
                            ),
                        )
                        msg = Twist()
                        linear = Vector3()
                        angular = Vector3()

                        linear.x = LIN_vel
                        linear.y = 0
                        linear.z = ANG_vel
                        angular.x = 0
                        angular.y = 0
                        angular.z = 0

                        msg.linear = linear
                        msg.angular = angular
                        cmd_pub.publish(msg)

                except Exception:
                    rospy.logerr("Error in Driver... Recovering.")
                    exc_type, exc_value, exc_traceback = sys.exc_info()
                    traceback.print_exception(
                        exc_type, exc_value, exc_traceback, limit=2, file=sys.stdout
                    )

                rospy.rostime.wallsleep(0.01)
        except Exception:
            rospy.logfatal("Driver Daemon crashed or halted!")

    def go_to_position(self, x, y):
        self.target_goal = (x, y, 0)
        self.flag = DriverStatus.PREEMPTING

    def update_pose(self, payload):
        self.pose = payload.pose

    def move(self, dl=0, da=0):
        msg = Twist()
        linear = Vector3()
        angular = Vector3()

        linear.x = dl
        linear.y = 0
        linear.z = 0
        angular.x = 0
        angular.y = 0
        angular.z = da

        msg.linear = linear
        msg.angular = angular

        self.cmd_vel_pub.publish(msg)

    def halt(self):
        self.latch = True
        self.flag = DriverStatus.HALT
        self.move()

    def get_current_loc(self):
        x, y, z, w = (
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        )
        t1 = 2.0 * (w * z + x * y)
        t2 = 1.0 - 2.0 * (y ** 2 + z ** 2)
        return (self.pose.position.x, self.pose.position.y, math.atan2(t1, t2))

    def is_target_reached(self):
        reached = False
        if self.target == (None, None):
            reached = True
        else:
            x, y, _ = self.get_current_loc()
            tx, ty = self.target
            reached = self.config["xy_tolerance"] >= math.sqrt(
                (x - tx) ** 2 + (y - ty) ** 2
            )

        if self.flag == DriverStatus.HALT:
            return True

        self.latch = reached
        return reached

    def percent_complete(self):
        if self.source is None or self.target is None:
            return 0
        else:
            sx, sy = self.source
            tx, ty = self.target
            cx, cy, _ = self.get_current_loc()
            return 100 * (
                1
                - (
                    math.sqrt((cx - tx) ** 2 + (cy - ty) ** 2)
                    / math.sqrt((sx - tx) ** 2 + (sy - ty) ** 2)
                )
            )

    def follow_path(self, path):
        msg = Path()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        if self.latch:
            self.source = path[0]
            self.target = path[-1]

        for i, (x, y) in enumerate(path):
            if i + 1 < len(path):
                nx, ny = path[i + 1]
                r = math.atan2(ny - y, nx - x)
            else:
                nx, ny = path[i - 1]
                r = math.atan2(y - ny, x - nx)

            pose = PoseStamped()
            pose.header = msg.header

            # set position
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.2

            # set orientation
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(r / 2)
            pose.pose.orientation.w = math.cos(r / 2)

            # add to path
            msg.poses.append(pose)

        self.path_pub.publish(msg)
