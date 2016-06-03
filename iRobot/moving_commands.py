#work in progress
import time
import serial


ports = ['/dev/ttyAMC2', '/dev/ttyUSB0']
for usbPort in ports:
	try:
		ser = serial.Serial(usbPort, baudrate = 9600)
	except:
		print "The port: " + usbPort + "did not work!"


def move_forward():
	sleep_time = 1
	acceleration = 10
	curr_speed = 0 #in mm/s
	max_speed = 200
	max_mouse = 15680 #raw serial number from mouse.ino for 14 inch movement
	deceleration_distance = 0.5 *  (((max_speed * sleep_time)**2) / acceleration)  
		#14 incehs, maximize speed, max speed at 7 inches, 
		#200 mm/s is max speed, accelerate slowly until reach max speed	
	while curr_speed < max_speed: 
		curr_speed += acceleration
		self.sendCommandASCII('145 ' + '''v high byte''' + '''v low byte''' + '255 255') #
		timer.sleep(sleep_time)

	#now at max speed, check total distance
	while goal_distance - ser.readline() > deceleration_distance:
	 	pass #do nothing, wait until within distance

	#at this point goal_distance - mouse_distance <= deceleration_distance:
	while curr_speed > 0:
	 	#decelerate
	 	curr_speed -= acceleration
	 	self.sendCommandASCII('145 ' + '''v high byte''' + '''v low byte''' + '255 255') #
		timer.sleep(sleep_time)

	 #congrats, you are now stopped 

def turn_left():
	x_max = 50 ####NEEED TO TESTTT
	
	self.sendCommandASCII('137 255 56 255 255') #turn counterclockwise/left

	while ser.readline() < x_max:
		time.sleep(1) #or just pass
	self.sendCommandASCII('137 00 00 00 00') #stop


def turn_right():
	x_max = -50

	self.sendCommandASCII('137 255 56 00 01') #turn clockwise
	#do i need to also accelerate the turns? assuming no
	while ser.readline() < x_max: #in case x and y values are individually slightly off...fucked if they ever are
		pass
	self.sendCommandASCII('137 00 00 00 00') #stop


def turn_around():
	x_max = 100 #(double turn left/right max)
	
	self.sendCommandASCII('137 255 56 255 255')

	while ser.readline() < x_max:
		time.sleep(1)
	self.sendCommandASCII('137 00 00 00 00')

def dec_to_two_comp_to_hex(number, bits = 16):
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value

        #need to now convert to hex
    return val           


def in_to_mm(numb):
	return numb * 25.4


def mm_to_in(numb):
	return numb / 25.4
