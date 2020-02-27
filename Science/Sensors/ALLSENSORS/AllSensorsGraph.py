import xlrd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure() #for temperature
ax1 = fig.add_subplot(1,1,1)

fig2 = plt.figure() #for moisture
ax2 = fig2.add_subplot(1,1,1)

fig3 = plt.figure() #for CO
ax3 = fig3.add_subplot(1,1,1)

fig4 = plt.figure() #for UV
ax4 = fig4.add_subplot(1,1,1)

def animateTemperature(i):
	book = xlrd.open_workbook("Science.xls")
	sheet = book.sheet_by_index(0)

	xs = []
        ys = []
	
	for i in range(sheet.nrows):
            xs.append(float(i))
            ys.append(float(sheet.row_values(i)[0][:-1]))
        ax1.clear()
        ax1.plot(xs, ys)
        ax1.set_title("Temperature(C)")

def animateMoisture(i):
	book = xlrd.open_workbook("Science.xls")
	sheet = book.sheet_by_index(0)

	xs = []
        ys = []
	
	for i in range(sheet.nrows):
            xs.append(float(i))
            ys.append(float(sheet.row_values(i)[1][:-1]))
        ax2.clear()
        ax2.plot(xs, ys)
        ax2.set_title("Moisture")

def animateCO(i):
	book = xlrd.open_workbook("Science.xls")
	sheet = book.sheet_by_index(0)

	xs = []
        ys = []
	
	for i in range(sheet.nrows):
            xs.append(float(i))
            ys.append(float(sheet.row_values(i)[2]))
        ax3.clear()
        ax3.plot(xs, ys)
        ax3.set_title("CO Value")

def animateUV(i):
	book = xlrd.open_workbook("Science.xls")
	sheet = book.sheet_by_index(0)

	xs = []
        ys = []
	
	for i in range(sheet.nrows):
            xs.append(float(i))
            ys.append(float(sheet.row_values(i)[3]))
        ax4.clear()
        ax4.plot(xs, ys)
        ax4.set_title("UV Value")

ani = animation.FuncAnimation(fig, animateTemperature, interval=1000)
ani2 = animation.FuncAnimation(fig2, animateMoisture, interval=1000)
ani3 = animation.FuncAnimation(fig3, animateCO, interval=1000)
ani4 = animation.FuncAnimation(fig4, animateUV, interval=1000)
plt.show()
