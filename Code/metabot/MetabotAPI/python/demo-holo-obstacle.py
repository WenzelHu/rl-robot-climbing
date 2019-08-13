#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# sudo PYTHONPATH=. python ../demo-holo-obstacle.py <bluetooth port>
 
from holobot import Holobot
import time
import sys

if len(sys.argv) != 2:
    print ("error: I need the bluetooth port")
    sys.exit(0)

holo = Holobot(sys.argv[1], 115200)
print("<tapper Ctrl-C pour arreter>")
holo.reset_yaw()

while holo.get_dist(0) > 10:
    holo.move_toward(-60,0)
holo.move_toward(20,0)
time.sleep(0.10)
holo.stop_all()
time.sleep(0.10)

def go_to_azimut(az):
    holo.turn(0)
    holo.move_toward(0,0)
    while holo.get_yaw() - az < 1:
        holo.turn(-20)
    while holo.get_yaw() - az > -1:
        holo.turn(20)
    print("az = " + str(holo.get_yaw()))

def optim_azimut():
    holo.turn(-20)
    holo.move_toward(0,0)
    min_d = 50
    best_az = -1
    for i in range(10):
        if min_d > holo.get_dist(0):
            min_d = holo.get_dist(0)
            best_az = holo.get_yaw();
        time.sleep(0.1)
    print('best az = ' + str(best_az))
    go_to_azimut(best_az)

t = 0
while True:
    try:
        if holo.get_dist(0) > 20:
            holo.move_toward(0,0)
            holo.turn(-20)
        elif holo.get_dist(0) <= 15:
            holo.turn(0)
            d_error = holo.get_dist(0) - 10
            holo.move_toward(30,-80 - d_error)
        elif holo.get_dist(0) <= 20:
            holo.move_toward(-20,0)
            holo.turn(0)
        if False and t > 10:
            optim_azimut()
            t = 0
        t += 0.10
        time.sleep(0.10)
    except KeyboardInterrupt:
        break
    
holo.stop_all()


