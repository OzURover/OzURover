import xlrd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')

fig = plt.figure() #for temperature
ax1 = fig.add_subplot(1,1,1)

fig2 = plt.figure() #for moisture
ax2 = fig2.add_subplot(1,1,1)

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

ani = animation.FuncAnimation(fig, animateTemperature, interval=1000)
ani2 = animation.FuncAnimation(fig2, animateMoisture, interval=1000)
plt.show()
