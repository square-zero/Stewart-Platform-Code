'''
Created on May 29, 2016

@author: Amanda Loh
'''
#imports needed for the module
import pygame #for the sound ability
pygame.init() #for the initialization of pygame
from Tkinter import *
import tkMessageBox
import tkSimpleDialog
import time
import serial
from time import sleep
from copy import copy, deepcopy
import struct
import sys, glob # for listing serial ports

#global variables
PORT = "/dev/ttyUSB0" #for connecting to the robot
connection = None
FLOORMAP =  [[000,000,000], \
             [000,000,000], \
             [000,000,000], \
             [000,000,000], \
             [000,000,000]] 
#import for Wavefront
from Wavefront import waveFrontPlanner

class AdaptiveMapping:
    #constructs an adaptive mapping object with the x and y position
    #kept track of that one is currently at and destination x and y
    def __init__(self, posX, posY, endX, endY):
        self.posX = posX
        self.posY = posY
        self.endX = endX
        self.endY = endY
        self.__onConnect() #connect to the robot #connect to robot
        
    def __onConnect(self):
        global connection
        if connection is not None:
            print "Oops', You're already connected!"
        else:
            port = PORT
            print "Trying " + str(port) + "... "
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print "Connected!"
                print "Connected.... Connection succeeded!"
                #set into passive mode
                self.sendCommandASCII('128')
                #set into save mode
                self.sendCommandASCII('131')
                #send a beep to make sure P/S protocol worked
                self.sendCommandASCII('140 3 1 64 16 141 3')
            except:
                print "Failed', Sorry, couldn't connect to " + str(port)
                
    #This is the main call for the adaptive mapping module from bartender to the customer
    #this is the call when the dirnk is delivered
    #The params in this method can be tuned to take in a different floormap, start and goal state
    def runAdaptiveMappingToCustomer(self):
        print "Start trip to customer..."
        path1 = self.adaptiveMapping(self.endX, self.endY, self.posX, self.posY, deepcopy(FLOORMAP), "South", deepcopy(FLOORMAP))
        if path1 is not None:
            self.sendCommandASCII('137 00 00 00 00') #stop
            pygame.mixer.music.load("DrinkArrived.mp3")
            pygame.mixer.music.play()
            time.sleep(6) #wait to say you're welcome
            pygame.mixer.music.load("YouWelcome.mp3")
            pygame.mixer.music.play()
            time.sleep(2)
            self.sendCommandASCII('137 00 00 00 00') #stop
    
    #This is the main call for the adaptive mapping module form customer to bartender            
    def runAdaptiveMappingToBartender(self):
        #now we need to do a roundtrip
        print 'Start trip back to bartender...'
        path2 = self.adaptiveMapping(self.posX, self.posY, self.endX, self.endY, deepcopy(FLOORMAP), "South", deepcopy(FLOORMAP))
        if path2 is not None:
            pygame.mixer.music.load("BartenderArrive.mp3")
            pygame.mixer.music.play()
            time.sleep(3)
            self.sendCommandASCII('137 00 00 00 00') #stop
    
    
    #This is the recursive adapting mapping function that recursively recomputes the wavefront algo until
    #a valid path is found 
    #Parameters
    #robotPosX = input of robot's current x coordinate position: int
    #robotPosY = input of robot's current y coordinate position: int
    #goalX = input of the robot's goal x coordinate position: int
    #goalY = input of the robot's goal y coordinate position: int
    #floorMap = input the modified grid of the room the robot will traverse (can be modified as obstacles are found): matrix ints
    #compass = input the direction of the robot: default is usually "South": string
    #origfloorMap = input the original grid of the room robot will traverse (need to keep track to "erase temp objects")
    #return nothing
    def adaptiveMapping(self, robotPosX, robotPosY, goalX, goalY, floorMap, compass, origFloorMap):
        if robotPosX == goalX and robotPosY == goalY:
            print "PATH COMPLETED! I got into base case"
            #never get in here actually
        else:
            planner = waveFrontPlanner(floorMap, False)
            planner.setRobotPosition(robotPosX, robotPosY)
            planner.setGoalPosition(goalX, goalY)
            path = planner.run(True)
            if path is None:
                #try to find path erasing all the temp obstacles!
                floorMap = origFloorMap #we need to assign the origFloorMap to floorMap var
                planner = waveFrontPlanner(origFloorMap, False)
                path = planner.run(True)
                if path is None:
                    print "I give up! There is no path GAHHHHHH"
                    pygame.mixer.music.load("BartenderSaveMe.mp3")
                    pygame.mixer.music.play()
                    time.sleep(5)
                    return None
            triTuple = self.tryDrive(robotPosX, robotPosY, goalX, goalY, floorMap, path, compass)
            if triTuple is not None:
                robotPosX = triTuple[0]
                robotPosY = triTuple[1]
                compass = triTuple[2]
                #tried to find another path
                pygame.mixer.music.load("ExcuseMe.mp3")
                pygame.mixer.music.play()
                time.sleep(5)
                self.adaptiveMapping(robotPosX, robotPosY, goalX, goalY, floorMap, compass, origFloorMap) #recursively call itself
            print "PATH COMPLETED!"
            return path
    
    #This is the method that determines if there is an obstacle in front of the robot or not 
    #Accepts no params but returns a boolean
    #True: if there is an obstacle in front
    #False: if there is no obstacle in front of robot
    #does this by making an array of read serial bits and then splitting on "." to get the numbers
    #takes the middle elements in array not corrupted and averages the 3 to get the value 
    def isThereObstacle(self):
        #These are the tune factors for the peripherals
        #For the peripherals that make sure width of robot doesn't hit anything the range = 47
        PERIPH = 49
        #For the inside sensors that detect objects right in front the range = 33
        INSIDE = 49
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM3']
        for usbPort in ports:
            try:
                ser = serial.Serial(usbPort, baudrate = 9600)
                print "The port: " + usbPort + "succeeded!"
            except:
                print "The port: " + usbPort + " did not work!"
        #These are the sums for each sensor
        sum1 = 0.0
        sum2 = 0.0
        sum3 = 0.0
        sum4 = 0.0
        
        #sleep for a minute so we can poll resl time values
        #time.sleep(5)
        n = 35
        print "Check to see if there is an obstacle now!!!"
        for _ in range(n):
            num1 = ser.readline()
            temp1 = ''.join(c for c in str(num1) if c.isdigit())
            while temp1 == '':
                num1 = ser.readline()
                temp1 = ''.join(c for c in str(num1) if c.isdigit())
            num2 = ser.readline()
            temp2 = ''.join(c for c in str(num2) if c.isdigit())
            while temp2 == '':
                num2 = ser.readline()
                temp2 = ''.join(c for c in str(num2) if c.isdigit()) 
            num3 = ser.readline()
            temp3 = ''.join(c for c in str(num3) if c.isdigit())
            while temp3 == '':
                num3 = ser.readline()
                temp3 = ''.join(c for c in str(num3) if c.isdigit()) 
            num4 = ser.readline()
            temp4 = ''.join(c for c in str(num4) if c.isdigit())
            while temp4 == '':
                num4 = ser.readline()
                temp4 = ''.join(c for c in str(num4) if c.isdigit())
            if int(temp1) >= 0: #make sure reading is not neg otherwise do not count
                sum1 += int(temp1)
            if int(temp2) >= 0:
                sum2 += int(temp2)
            if int(temp3) >= 0:
                sum3 += int(temp3)
            if int(temp4) >= 0:
                sum4 += int(temp4)
        #compute the averages after taking n samples
        final1 = sum1/n
        final2 = sum2/n 
        final3 = sum3/n
        final4 = sum4/n     
           
        print "final 1 is: " + str(final1)
        print "final 2 is: " + str(final2)
        print "final 3 is: " + str(final3)
        print "final 4 is: " + str(final4)
        if final1 < 0 or final2 < 0 or final3 < 0 or final4 < 0:
            print "no obstacle"
            return False
        if final1 < PERIPH or final2 < PERIPH or final3 < INSIDE or final4 < INSIDE: #these are the ones farther away: peripherals
            print "OBSCACLEEEE"
            return True

    #this is actually the drive module that actually sends command to the robot: before the robot moves forward,
    #the sensors will check to see if there is an obstacle
    #if there is one, the current location of the robot is returned as well as its orientation so the wavefront algo
    #can be run again
    #Parameters
    #robotPosX = input of robot's current x coordinate position: int
    #robotPosY = input of robot's current y coordinate position: int
    #goalX = input of the robot's goal x coordinate position: int
    #goalY = input of the robot's goal y coordinate position: int
    #floorMap = input the grid of the room the robot will traverse (can be modified as obstacles are found): matrix ints
    #compass = input the direction of the robot: default is usually "South": string 
    #robotPosX, robotPosY, goalX, goalY, floorMap, path, compass
    def tryDrive(self, robotPosX, robotPosY, goalX, goalY, floorMap, path, compass):
        
        #compass = "South" #what direction is it facing
        prevX = robotPosX
        prevY = robotPosY
        print "I started at: " + str(prevX) + "," + str(prevY)
        for listXYs in path:
            currX = listXYs[0]
            currY = listXYs[1]
            print "Going to: " + str(currX) + "," + str(currY)
            
            #There are four cases of places you could go
            
            #GOING NORTH
            if (prevX - 1 == currX):
                #need to check if there is something in square
                
                if compass == "South":
                    mc.turn_around() #turning around
                elif compass == "East":
                    mc.turn_left()
                elif compass == "West":
                    mc.turn_right()
                compass = "North"
            #GOING SOUTH
            elif (prevX + 1 == currX):
                if compass == "North":
                    #want to rotate 180 degrees
                    mc.turn_around()
                elif compass == "East":
                    mc.turn_right()
                elif compass == "West":
                    mc.turn_left()
                compass = "South"
            #GOING WEST
            elif (prevY - 1 == currY):
                if compass == "South":
                    mc.turn_right()
                elif compass == "North":
                    mc.turn_left()
                elif compass == "East":
                    mc.turn_around()
                compass = "West"
            #GOING EAST   
            elif (prevY + 1 == currY):
                if compass == "South":
                    mc.turn_left()
                elif compass == "North":
                    mc.turn_right()
                elif compass == "West":
                    #want to rotate 180 degrees
                    mc.turn_around()
                compass = "East"
            #self.sendCommandASCII('137 00 00 00 00') #...stop build in to mc. methods

            #check if you can drive forward
            if self.isThereObstacle():
                #tell the robot to stop cuz you gon hit somethin...
                if compass == "East":
                    floorMap[prevX][prevY + 1] = 999 #mark as obstacle
                elif compass == "West":
                    floorMap[prevX][prevY - 1] = 999 #mark as obstacle
                elif compass == "North":
                    floorMap[prevX - 1][prevY] = 999 #mark as obstacle
                elif compass == "South":
                    floorMap[prevX + 1][prevY] = 999 #mark as obstacle  
                print prevX
                print prevY
                print compass
                return prevX, prevY, compass #return the current robot's location and compass
            #after each iteration wanna drive forward

           '''/\ WHAT DOES THIS SECTION DO?!?'''

            self.sendCommandASCII('137 00 148 80 00') #drive: forward
            time.sleep(1)#pause
            self.sendCommandASCII('137 00 188 80 00') #drive
            time.sleep(1)#pause 
            #update the prev to the currX     
            



            prevX = currX
            prevY = currY
            if prevX == goalX and prevY == goalY: #if we are at goal we need to re-calibrate to always start South!!!
                if compass == "North":
                    mc.turn_around()
                elif compass == "East":
                    mc.turn_right()
                elif compass == "West":
                    mc.turn_left()
                compass = "South"
                #self.sendCommandASCII('137 00 00 00 00') #stop built in already
    
    #All Methods NEEDED FOR THE IROBOT INTEGRATION/DRIVER   
    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
    def sendCommandASCII(self, command):
        cmd = ""
        for v in command.split():
            cmd += chr(int(v))

        self.sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                connection.write(command)
            else:
                tkMessageBox.showerror('Not connected!', 'Not connected to a robot!')
                print "Not connected."
        except serial.SerialException:
            print "Lost connection"
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            
        print ' '.join([ str(ord(c)) for c in command ])
        '''
        self.text.insert(END, ' '.join([ str(ord(c)) for c in command ]))
        self.text.insert(END, '\n')
        self.text.see(END)
        '''

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        global connection
        
        try:
            return struct.unpack(fmt, connection.read(n))[0]
        except serial.SerialException:
            print "Lost connection"
            tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print "Got unexpected data from serial port."
            return None
            
     def move_forward(self):
        sleep_time = .001 #in seconds
        acceleration = 1 #in mm/s/s
        curr_speed = 0 #in mm/s
        max_speed = 400 #in mm/s
        # goal_distance = 15680 #raw serial number from mouse.ino for 14 inch movement
        # deceleration_distance = 0.5 *  (((max_speed * sleep_time)**2) / acceleration)  
            #14 incehs, maximize speed, max speed at 7 inches, 
            #200 mm/s is max speed, accelerate slowly until reach max speed    
    
        for i in range(max_speed): #accelerate 1mm/s every ms
            curr_speed += acceleration
            # Ryan's edit to convert curr_speed to high and low bytes
#             (high, low) = bytes(curr_speed)
            (high, low) = divmod(curr_speed, 0x100)
            self.sendCommandASCII('145 ' + str(high) + " " + str(low) + " " + str(high) + " " + str(low)) #
            time.sleep(sleep_time)
    
        # accel_distance = ser.readline()    
    
        #now at max speed, check total distance
        # while goal_distance - ser.readline() > accel_distance:
        #      pass #do nothing, wait until within distance
        time.sleep(0.12) #CHANGE THIS VALUE WHEN TUNING
    
        #at this point goal_distance - mouse_distance <= deceleration_distance:
        for i in range(max_speed):
             #decelerate
            curr_speed -= acceleration
            (high, low) = divmod(curr_speed, 0x100)
            self.sendCommandASCII('145 ' + str(high) + " " + str(low) + " " + str(high) + " " + str(low)) #
            time.sleep(sleep_time)
        
        self.sendCommandASCII('137 00 00 00 00') #stop
         #congrats, you are now stopped 

    def turn_left(self):
        #x_max = 50
        
        self.sendCommandASCII('137 255 56 255 255') #turn counterclockwise/left
    
        time.sleep(0.95)  #CHANGE THIS VALUE WHEN TUNING
        # while ser.readline() < x_max:
        #     time.sleep(1) #or just pass
        self.sendCommandASCII('137 00 00 00 00') #stop
    
    
    def turn_right(self):
        # x_max = -50
    
        self.sendCommandASCII('137 255 56 00 01') #turn clockwise
        
        time.sleep(0.95)  #CHANGE THIS VALUE WHEN TUNING
    
        # while ser.readline() < x_max: #in case x and y values are individually slightly off...fucked if they ever are
        #     pass
        self.sendCommandASCII('137 00 00 00 00') #stop
    
    
    def turn_around(self):
        # x_max = 100 #(double turn left/right max)
        
        self.sendCommandASCII('137 255 56 255 255')
    
        time.sleep(1.9) #CHANGE THIS VALUE WHEN TUNING
        # while ser.readline() < x_max:
        #     time.sleep(1)
        self.sendCommandASCII('137 00 00 00 00')
#     
#     def dec_to_two_comp_to_hex(number, bits = 16):
#         if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
#             val = val - (1 << bits)        # compute negative value
#     
#             #need to now convert to hex
#         return val           
   
