#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# sudo PYTHONPATH=. python ../demo-holo-obstacle.py <bluetooth port>
 
from holobot import Holobot
import time
import math
import sys

if len(sys.argv) != 2:
    print ("error: I need the bluetooth port")
    sys.exit(0)

holo = Holobot(sys.argv[1], 115200)

print("<tapper Ctrl-C pour arreter>")
holo.reset_yaw()

etat = 'attendre'

while True:
    try:
        t = holo.get_time()
        if etat == 'attendre':
            pousse = holo.get_wheel_speeds(0) != 0 or holo.get_wheel_speeds(1) != 0 or holo.get_wheel_speeds(2) != 0
            if pousse:
                etat = 'pousse_reaction'
        if etat == 'pousse_reaction':
            # TODO: determiner la direction des roues et pousser dans ce sens
            holo.beep(880,200)
            dt = 0.05
            for n in range(20):
                holo.turn(30*math.sin(10*n*dt))
                time.sleep(dt)
            holo.stop_all()
            time.sleep(1)
            etat = 'attendre'

        # TODO: si on s'approche des capteurs de distances, le robot s'éloigne
        
        time.sleep(0.05)
    except KeyboardInterrupt:
        break

holo.stop_all()
time.sleep(0.10)




