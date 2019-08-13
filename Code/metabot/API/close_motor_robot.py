# To build the Metabot module, please refer to the project README
from metabot import MetabotV2
import time

metabot = MetabotV2('/dev/rfcomm3')

# Monitoring (otherwise the read commands won't work)
metabot.monitor(10)
metabot.waitUpdate();

# Reset
print('')
print('> Reset')
metabot.reset()
time.sleep(4)

print('> Stopping')
metabot.stop()


