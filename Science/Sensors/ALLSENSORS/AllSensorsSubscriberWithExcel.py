#ADD ANEMOMETER DATA
#ADD BMP180 DATA

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import xlwt
from datetime import datetime

wb = xlwt.Workbook()
ws = wb.add_sheet("ScienceData")

line = 0

dataCounter = 0
datas = [0,0,0,0]

def write(data):
    global line, dataCounter, datas

    if "+" in data.data:
        datas[dataCounter] = data.data.split("+")[1]
    else:
        datas[dataCounter] = data.data
    
    dataCounter +=1
    if dataCounter == 4:
        dataCounter = 0

    if dataCounter == 0:

        temperature = datas[0]
        moisture = datas[1]
        CO = datas[2]
        UV = datas[3]
        cur_time = datetime.now().strftime("%H:%M:%S")

        print(datas)

        ws.write(line, 0, temperature)
        ws.write(line, 1, moisture)
        ws.write(line, 2, CO)
        ws.write(line, 3, UV)
        ws.write(line, 4, cur_time)

        line+=1
        wb.save("Science.xls")

def read():
    rospy.init_node("ScienceSensorsChannel", anonymous=True)
    rospy.Subscriber("AllSensors", String, write)
    rospy.spin()

if __name__ == "__main__":
    read()