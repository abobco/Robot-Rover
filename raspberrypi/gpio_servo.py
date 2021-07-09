from gpiozero import Servo
from time import sleep
from gpiozero.tools import sin_values
from gpiozero.pins.pigpio import PiGPIOFactory

myCorrection=0.45
maxPW=(2.0+myCorrection)/1000
minPW=(1.0-myCorrection)/1000

servo = Servo(12, min_pulse_width=minPW,max_pulse_width=maxPW,  pin_factory = PiGPIOFactory())

from gpiozero import Servo
from time import sleep
 

servo.mid()

# # Adjust the pulse values to set rotation range
# min_pulse = 0.000544	# Library default = 1/1000
# max_pulse = 0.0024		# Library default = 2/1000
# # Initial servo position
# pos = 0
# test = 0
 
# def delays(pos):
# 	if pos == 0:
# 		print('Stop', pos)
# 		sleep(3)
# 	elif pos == 1.0:
# 		print('Full Left', pos)
# 		sleep(3)
# 	elif pos == -1.0:
# 		print('Full Right', pos)
# 		sleep(3)
# 	else:
# 		print(pos)
# 		sleep(1)
 
# while True:
# 	# For statement example
# 	for pos in range(0, 20):
# 		pos = pos * 0.1 - 1
# 		servo.value = pos
# 		delays(pos)
 
# 	for pos in range(20, -1, -1):
# 		pos = pos * 0.1 - 1
# 		servo.value = pos
# 		delays(pos)