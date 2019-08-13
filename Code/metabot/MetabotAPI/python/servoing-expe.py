#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# PYTHONPATH=. python ../servoing-expe.py
 
from holobot import Holobot
import time
import sys

#holo = Holobot('/dev/tty.holo-DevB', 115200)
holo = Holobot('/dev/rfcomm11', 115200)

speed=float(sys.argv[1])
d=1
period=2
holo.debug_state(1)
curr_t = -1
while True:
    try:
        if holo.get_time() > curr_t + period:
            holo.turn((2*d-1)*speed)
            d=1-d
            curr_t = holo.get_time()
        time.sleep(0.01)
    except KeyboardInterrupt:
        break
holo.debug_state(0)
holo.turn(0)

