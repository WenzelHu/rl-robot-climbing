#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# PYTHONPATH=. python ../demo-holo-monitor.py
 
import matplotlib.pyplot as plt
from holobot import Holobot
import time
import math
import sys

if len(sys.argv) != 2:
    print ("error: I need the bluetooth port")
    sys.exit(0)

holo = Holobot(sys.argv[1], 115200)
print("- prise de mesure en cours")
print("<tapper Ctrl-C pour arreter>")

holo.reset_yaw()
distances = []
while True:
    holo.turn(90)
    try:
        d = holo.get_dist(1)
        d = 12.0 + (d-7.0)*8.0/10.0
        if d <= 20:
            t = holo.get_time()
            az = holo.get_yaw()
            while az < -180.0: az = az + 360.0
            while az > 180.0: az = az - 360.0
            distances.append([az, holo.get_dist(1)+3.0])
        time.sleep(0.05)
    except KeyboardInterrupt:
        break

holo.stop_all()

plt.grid()
plt.xlim([-50,50])
plt.ylim([-50,50])

X = [0.0]
Y = [0.0]
for c in distances:
    a = c[0]*math.pi/180.0
    d = c[1]
    X.append(d * math.cos(-a))
    Y.append(d * math.sin(-a))

plt.scatter(X,Y)
plt.show()
