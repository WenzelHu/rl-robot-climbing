# To build the Metabot module, please refer to the project README
from metabot import MetabotV2
import time

metabot = MetabotV2('/dev/rfcomm0')

# Monitoring (otherwise the read commands won't work)
metabot.monitor(10)
metabot.waitUpdate();

# Starting motors

print('')
print('> Starting the motors')
metabot.start()
time.sleep(3);


'''
# Sound
print('')
print('> Beep')
metabot.beep(440, 500)
time.sleep(3)
'''

for i in range(30) :
    print('Motor positions:' + str(metabot.motors))
    time.sleep(0.1)

# set motor values
print('')
print('> Setting motor values')


metabot.setMotors(0.0, 0.0, 0.0, 0.0, -29.76, -29.76, -29.76, -29.76, -123.87, -123.87, -123.87, -123.87)
time.sleep(0.5)

#metabot.setMotors(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)


# Read motor values
for i in range(30) :
    print('Motor positions:' + str(metabot.motors))
    time.sleep(0.5)
# Reset
print('')
print('> Reset')
metabot.reset()
time.sleep(4)

print('> Stopping')
metabot.stop()


