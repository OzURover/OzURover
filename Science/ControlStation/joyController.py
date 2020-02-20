import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

drillOpen = 0

def joyCallback(data):
    global drillOpen

    message = Int32()

    if data.buttons[8] == 1:

        #Actuator of drill
        if data.axes[4] < -0.1:
            message.data = -1
            pubDrillAct.publish(message)

        elif data.axes[4] > 0.1:
            message.data = 1
            pubDrillAct.publish(message)

        #Drill spin on-off
        elif data.buttons[7] == 1:
            if drillOpen == 0:
                drillOpen = 1
            else:
                drillOpen = 0
            message.data = drillOpen
            pubDrill.publish(message)

        #Actuator of sensors
        elif data.axes[1] < -0.1:
            message.data = -1
            pubSensorAct.publish(message)
        
        elif data.axes[1] > 0.1:
            message.data = 1
            pubSensorAct.publish(message)

        #Stepper motor left-right-shuffle
        elif data.axes[6] == -1:
            message.data = 1
            pubStep.publish(message) #right
        
        elif data.axes[6] == 1:
            message.data = -1
            pubStep.publish(message) #left

        elif data.buttons[6] == 1:
            message.data = 2
            pubStep.publish(message) #suffle

        #water pump 1
        elif data.buttons[3] == 1:
            message.data = 1
            pubW.publish(message)
        
        #water pump 2
        elif data.buttons[1] == 1:
            message.data = 2
            pubW.publish(message)
        
        #water pump 3
        elif data.buttons[0] == 1:
            message.data = 3
            pubW.publish(message)

        #water pump 4
        elif data.buttons[2] == 1:
            message.data = 4
            pubW.publish(message)

        else:
            message.data = 0

            pubDrillAct.publish(message)
            pubSensorAct.publish(message)


    else:
        message.data = 0
        
        pubDrillAct.publish(message)
        pubSensorAct.publish(message)

def start():
    global pubDrillAct, pubDrill, pubW1, pubW2, pubW3, pubW4, pubSensorAct, pubStep
    
    rospy.init_node("ScienceControl")

    pubDrillAct = rospy.Publisher("drillActuator", Int32, queue_size=5)
    pubDrill = rospy.Publisher("drill", Int32, queue_size=5)
    pubW = rospy.Publisher("water", Int32, queue_size=5)
    pubSensorAct = rospy.Publisher("sensorActuator", Int32, queue_size=5)
    pubStep = rospy.Publisher("stepper", Int32, queue_size=5)

    rospy.Subscriber("joy", Joy, joyCallback)

    rospy.spin()

if __name__ == "__main__":
    start()