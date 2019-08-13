#!/usr/bin/python
# coding: utf8

# To build the Holobot module, please refer to the project README
# command to launch the demo :
# sudo PYTHONPATH=. python ../demo-holo-monitor.py <bluetooth port>
 
from holobot import Holobot
import time
import sys

if len(sys.argv) != 2:
    print ("error: I need the bluetooth port")
    sys.exit(0)

holo = Holobot(sys.argv[1], 115200)
holo.calibrate_magneto()
time.sleep(10)

holo.debug_state(1)
while True:
    try:
        time.sleep(1)
    except KeyboardInterrupt:
        break
holo.debug_state(0)

