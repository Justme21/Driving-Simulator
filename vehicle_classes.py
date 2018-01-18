import math
import random

#Dimensions are those of a "Ford Focus 5-door 1.8i Zetec" as found at
# http://www.metric.org.uk/motoring  . Units are metres
car_length = 4.17 
car_width = 1.7

class Car():
    def __init__(self,lane,label,timestep=.1):
        #x and y of the centre of mass (COM) of the car.
        # For simplicity we assume the COM is the centre of the car
        self.x_com = None
        self.y_com = None
  
        #Velocity of the car
        self.v = None

        #Heading of the car (direction it is facing in degrees)
        self.heading = None

        #Dimensions of the car
        self.length = car_length
        self.width = car_width

        #The object (lane or junction) that the car is on
        self.on = None

        #The processing speed of the car (how often it changes it's action)
        self.timestep = timestep
        
        #Sense Variables
        self.on_road = True #Check if car still on road

        #Initialise the position, velocity and heading features
        self.init_setup(lane)
        
        self.label = "C{}".format(label)


    def init_setup(self,lane):
        self.on = lane
        lane.on = self

        self.on_road = True

        self.heading = lane.direction

        x_disp = lane.width/2
        y_disp = (self.length/2)+ random.random()*(lane.length-self.length)
        phi = math.sqrt(x_disp**2 + y_disp**2)
        delta = math.degrees(math.atan(x_disp/y_disp))

        mod_x,mod_y = 1,1

        #Lane is up/down
        if (lane.direction in range(46,135)) or (lane.direction in range(226,315)):
            if lane.direction in range(46,135): mod_x *= -1
            else: mod_y *= -1
            omega = lane.direction+delta
        #Lane is left/right
        else:
            if lane.direction in range(135,226): mod_x *= -1
            else: mod_y *= -1
            omega = lane.direction-delta

        if lane.direction in range(46,135): mod_x = -1; mod_y = 1
        

        self.y_com = lane.y+mod_y*phi*math.sin(math.radians(omega))
        self.x_com = lane.x+mod_x*phi*math.cos(math.radians(omega))
        
        #NOTE: In the future this should be changed so that intial speed is something
        #      reasonable and not just arbitrarily selected.
        self.v = 5.5 #19.8km/h units are metres per second 


    def move(self,accel,turn_angle):
        x_dot = self.v*math.cos(math.radians(turn_angle))
        y_dot = self.v*math.sin(math.radians(turn_angle))
        head_dot = turn_angle
        v_dot = accel
        
        self.y_com += self.timestep*y_dot
        self.x_com += self.timestep*x_dot
        self.v += self.timestep*v_dot
        self.heading += self.timestep*head_dot


    #def checkPositState(self):
        #if self.


    def sense(self):
        self.checkPositState()


    def print_status(self,mod=""):
        print("{}{}\tON: {}\tHEAD: {}\tSPEED: {}\tCOM: ({},{})".format(mod,\
               self.label,self.on.label,self.heading,self.v,round(self.x_com,2),\
               round(self.y_com,2)))
