#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# sudo PYTHONPATH=. python ../demo-holo-motion.py <bluetooth path>

from holobot import Holobot
import time
import sys
import math

if len(sys.argv) != 2:
    print ("error: I need the bluetooth port")
    sys.exit(0)

holo = Holobot(sys.argv[1], 115200)
speed = 100

holo.beep(880, 200)
    
## 4 directions ##

for angle in [0, 90, 180, -90]:
    holo.move_toward(speed, angle)
    time.sleep(1.0)
    holo.stop_all()
    time.sleep(0.25)
    holo.move_toward(-speed, angle)
    time.sleep(1.0)
    holo.stop_all()
    time.sleep(0.25)

## Rotation ##
        
holo.control(0,0,speed)
time.sleep(1)
holo.stop_all()
time.sleep(0.250)
holo.control(0,0,-speed)
time.sleep(1)

## Cercle ##

t = 0.0
dt = 0.050
while t < 4.0:
    x = math.sin(2*math.pi * (t/4) - math.pi/2)
    y = math.sin(2*math.pi * (t/4))
    holo.control(50*x,50*y,0)
    t += dt    
    time.sleep(dt) # 20Hz

holo.stop_all()
time.sleep(1.0)
## Mouvement combiné ##

t = 0.0
dt = 0.050
dir = 0
while t < 4.0:
    if (int(0.5*t)) % 2: rot = 23
    else: rot = -23
    holo.turn(rot)
    dir += rot*dt

    corr_f = 4.2
    if t < 2.0:
        holo.move_toward(speed, -corr_f*dir)
    else:
        holo.move_toward(-speed, -corr_f*dir)
    # z = math.sin(2*math.pi * t/1.5)
    # holo.turn(speed*z)
    t += dt
    time.sleep(dt) # 20Hz
    
## Epilogue ##
holo.beep(440,200)

