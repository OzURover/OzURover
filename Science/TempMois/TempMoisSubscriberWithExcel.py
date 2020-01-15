#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import xlwt
from datetime import datetime

wb = xlwt.Workbook()
ws = wb.add_sheet("TempMoisData")

line = 0

def write(data):
    global line

    temperature = data.data.split(",")[0]
    moisture = (data.data.split(",")[1]).split("+")[1]
    cur_time = datetime.now().strftime("%H:%M:%S")

    print("Temperature: " + temperature + "\n" + "Moisture: "+ moisture + "\n" + "-"*30)

    ws.write(line, 0, temperature)
    ws.write(line, 1, moisture)
    ws.write(line, 2, cur_time)

    line += 1
    wb.save("Science.xls")

def read():
    rospy.init_node("TempMoisChannel", anonymous=True)
    rospy.Subscriber("TempMois", String, write)
    rospy.spin()

if __name__ == '__main__':
    read()
