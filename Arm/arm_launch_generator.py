import sys
import serial.tools.list_ports

def print_node(f, name, port, baud):
	tempStr = ""
	tempStr += "<node pkg=\"rosserial_python\" type=\"serial_node.py\" "
	tempStr += "name=\"" + name + "\" output=\"screen\">"
	f.write(tempStr)
	tempStr = "\n<param name=\"port\" value=\"" + port + "\" />"
	f.write(tempStr)
	tempStr = "\n<param name=\"baud\" value=\"" + baud + "\" />"
	f.write(tempStr)
	tempStr = "\n</node> \n"
	f.write(tempStr)
	return tempStr

def startLaunch(f):
	tempStr = "<launch> \n"
	f.write(tempStr)

def endLaunch(f):
	tempStr = "\n</launch> \n"
	f.write(tempStr)

def print_dynamixel(f):
	tempStr = "\n<node name=\"dxl\" pkg=\"ozuRover2020\" type=\"dxl.py\" output=\"screen\">"
	f.write(tempStr)
	tempStr = "\n</node>"
	f.write(tempStr)
print("Auto launch file generator initiated!")
fid = open(sys.argv[1] + ".launch", "w")
startLaunch(fid)

ports = list(serial.tools.list_ports.comports())
dyndescription = 'FT232R USB UART'
armdescription = 'USB-Serial Controller'
armcount = 0

for p in ports:
	if p.description == armdescription:
		name = "arm_joint" + str(armcount)
		baud = "76800"
		port = p.device
		print_node(fid, name, port, baud)
		armcount += 1

print_dynamixel(fid)
endLaunch(fid)

fid.close()

print("Auto launch file generator successfully completed!")

