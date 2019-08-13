from metabot import MetabotV2
import time

class metabotAPI:
	def __init__(self):
		self.metabot = MetabotV2('/dev/rfcomm3')

		# Monitoring (otherwise the read commands won't work)
		self.metabot.monitor(10)
		self.metabot.waitUpdate();

		print('')
		print('> Starting the motors')
		self.metabot.start()
		time.sleep(2)

		print('Initial motor positions:' + str(self.metabot.motors))

		

	# motor go to the values that I send to them
	def motor_go_myway(self, motor_values):
		#print('Motor goes to desired inital position:')
		#print(motor_values)
		self.metabot.setMotors(motor_values[0], motor_values[1], motor_values[2], motor_values[3], motor_values[4], motor_values[5], motor_values[6], motor_values[7], motor_values[8], motor_values[9], motor_values[10], motor_values[11])
		#time.sleep(0.02)
		#print('Current motor positions:' + str(metabot.motors))
