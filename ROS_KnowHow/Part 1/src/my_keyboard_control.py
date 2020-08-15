#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

from std_srvs.srv import Empty
from turtlesim.srv import Spawn

import curses

def add_another_turtle(turtle_name):
    global publishers
    rospy.wait_for_service('spawn')
    spawn = rospy.ServiceProxy('spawn', Spawn)
    spawn_info = spawn(5, 5, 1.57, turtle_name)

    new_turtle_pub = rospy.Publisher(turtle_name+"/cmd_vel", Twist, queue_size=10)
    publishers.append(new_turtle_pub)

    return spawn_info

def reset_canvas():
    rospy.wait_for_service('reset')
    reset = rospy.ServiceProxy('reset', Empty)
    reset()
    

def twister(trans, rot):
    msg = Twist()
    msg.linear.x = trans
    msg.angular.z = rot

    return msg

def change_background(r, g, b):
    rospy.set_param('turtlesim/background_r', r)
    rospy.set_param('turtlesim/background_g', g)
    rospy.set_param('turtlesim/background_b', b)

    reset_canvas()

def keyboard_control():
    global publishers, pub
    
    k=0
    stdscr = curses.initscr()
    stdscr.clear()
    stdscr.refresh()

    current_turtle = 0

    while (k != ord('q')):
        stdscr.clear()
        angular, linear = 0, 0

        stdscr.addstr(0, 0, "Turtlebot OzuRover Control | Q for Quit")
        stdscr.addstr(1, 0, "R - Rotate | W - Forward | A - Reset | K - Add Turtle | L - Switch")

        if(k == ord('r')):
            stdscr.addstr(4, 0, "Rotating")
            angular = 1.8

        elif(k == ord('w')):
            stdscr.addstr(4, 0, "Forward")
            linear = 2.0

        elif(k == ord('k')):
            stdscr.addstr(4, 0, "Adding another turtle")
            turtle_name = "turtle"+str(len(publishers)+1)
            
            info = add_another_turtle(turtle_name)
            stdscr.addstr(6, 0, str(info))
        
        elif(k == ord('l')):
            current_turtle = ((current_turtle+1) % len(publishers))
            stdscr.addstr(7, 0, ("Selected Turtle: "+str(current_turtle)))

        elif(k == ord('c')):
            change_background(0,0,0)

        elif(k == ord('a')):
            reset_canvas()

            current_turtle = 0
            publishers = []
            publishers.append(pub)

        msg = twister(linear, angular)
        publishers[current_turtle].publish(msg)

        stdscr.refresh()

        k = stdscr.getch()

    stdscr.keypad(0)
    curses.echo()
    curses.nocbreak()
    curses.endwin()

def auto_motion():
    global pub
    rate = rospy.Rate(10)
    linear, angular = 0, 0.5
    counter = 0
    while not rospy.is_shutdown():
        linear += counter
        msg = twister(linear, angular)
        pub.publish(msg)
        rate.sleep()
        counter += 0.001

def twister_init():
    global pub, publishers
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    publishers = []
    publishers.append(pub)
    rospy.init_node('turtle_zoom_keyboard', anonymous=True)
    
    reset_canvas()

    # Control the turtle(s) with the keyboard
    keyboard_control()

    # Set linear and angular velocities and publish
    # auto_motion()

if __name__ == '__main__':
    try:
        twister_init()
    except rospy.ROSInterruptException:
        pass