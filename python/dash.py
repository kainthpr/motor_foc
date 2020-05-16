#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Test the speed of rapidly updating multiple plot curves
"""


import time as pytime

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import serial 
import collections


app = QtGui.QApplication([])

sp = serial.Serial(port = "COM8", baudrate=19200, bytesize=8, timeout=0, stopbits=serial.STOPBITS_ONE)

p = pg.plot()
p2 = pg.plot()
p3 = pg.plot()
p.setWindowTitle('pyqtgraph example: MultiPlotSpeedTest')
#p.setRange(QtCore.QRectF(0, -10, 5000, 20)) 
p.setLabel('bottom', 'Index', units='B')

nPlots = 1
nSamples = 100
#curves = [p.plot(pen=(i,nPlots*1.3)) for i in range(nPlots)]

c = pg.PlotCurveItem()
p.addItem(c)
p.setYRange(0, 60000)


c2 = pg.PlotCurveItem()
p2.addItem(c2)

c3 = pg.PlotCurveItem()
p3.addItem(c3)

lastTime = time()
fps = None
count = 0
# data = [0]*300
data = collections.deque(maxlen=nSamples)
data2 = collections.deque(maxlen=nSamples)
data3 = collections.deque(maxlen=nSamples)
data4 = collections.deque(maxlen=nSamples)
data5 = collections.deque(maxlen=nSamples)
data6 = collections.deque(maxlen=nSamples)
data7 = collections.deque(maxlen=nSamples)

def update():
    global c, data, p, lastTime, fps, nPlots, count
    # count += 1
    # data[np.random.randint(0, 300)] = np.random.random()

    c.setData(list(data7))
    c2.setData(list(data2))
    c3.setData(list(data6))


    now = time()
    dt = now - lastTime
    lastTime = now
    if fps is None:
        fps = 1.0/dt
    else:
        s = np.clip(dt*3., 0, 1)
        fps = fps * (1-s) + (1.0/dt) * s
    p.setTitle('%0.2f fps' % fps)
    app.processEvents()  ## force complete redraw for every plot

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(2)


class AThread(QtCore.QThread):
    def run(self):
        while 1:
            line = sp.readline()

            if line != "":
                try:
                    # print line
                    l = line.split(",")
                    if len(l) != 9:
                        continue
                    vel = float(l[0])
                    error = float(l[1])
                    integral = float(l[2])
                    angle = float(l[3])
                    rotor_angle = float(l[4])
                    m = float(l[5])
                    # adc = int(l[6])+int(l[7])+int(l[8].strip())
                    adc = int(l[8].strip())

                    data.append(vel)
                    data2.append(error)
                    data3.append(integral)
                    data4.append(angle)
                    data5.append(rotor_angle)
                    data6.append(m)
                    data7.append(adc)
                    print "{:3},   {:3},   {:3}".format(int(l[6]), int(l[7]), int(l[8].strip()))
                except:
                    pass
t1 = AThread()
t1.start()
    
## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        app.exec_()
    
