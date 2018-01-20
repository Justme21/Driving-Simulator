import math
import random

#Dimensions are those of a "Ford Focus 5-door 1.8i Zetec" as found at
# http://www.metric.org.uk/motoring  . Units are metres
car_length = 4.17 
car_width = 1.7

class Car():
    def __init__(self,road,is_top_lane,label,timestep=.1):
        #x and y of the centre of mass (COM) of the car.
        # For simplicity we assume the COM is the centre of the car
        self.x_com = None
        self.y_com = None

        self.four_corners = {"front_left":(),"front_right":(),"back_left":(),\
                             "back_right":()}
  
        #Velocity of the car
        self.v = None

        #Heading of the car (direction it is facing in degrees)
        self.heading = None

        #Dimensions of the car
        self.length = car_length
        self.width = car_width

        #The object (lane or junction) that the car is on
        #This is a list as the car might be straddling a lane and junction
        #Or two lanes
        self.on = None

        #The processing speed of the car (how often it changes it's action)
        self.timestep = timestep
        
        #Sense Variables
        self.on_road = True #Check if car still on road
        self.crashed = False #Check if car has crashed into something

        #Initialise the position, velocity and heading features
        self.initSetup(road,is_top_lane)
        
        self.label = "C{}".format(label)


    def setFourCorners(self):
        """Compute the initial coordinates of the four corners of the car (with respect
           to the heading). These values are stored and continuously updated in a 
           dictionary for ease of identifying crashes later on"""
        direction = self.heading
        phi = math.sqrt((self.length/2)**2 + (self.width/2)**2)
        delta = math.degrees(math.atan(self.width/self.length))
        self.four_corners["front_left"] = (self.x_com+phi*math.cos(math.radians(\
                                           direction+delta)),self.y_com-phi*math.sin(\
                                           math.radians(direction+delta)))
        self.four_corners["front_right"] = (self.x_com+phi*math.cos(math.radians(\
                                           direction-delta)),self.y_com-phi*math.sin(\
                                           math.radians(direction-delta)))
        self.four_corners["back_left"] = (self.x_com-phi*math.cos(math.radians(\
                                           direction-delta)),self.y_com+phi*math.sin(\
                                           math.radians(direction-delta)))
        self.four_corners["back_rightt"] = (self.x_com-phi*math.cos(math.radians(\
                                           direction+delta)),self.y_com+phi*math.sin(\
                                           math.radians(direction+delta)))


    def initSetup(self,road,is_top_lane):
        """Performing initial setup of the car. Input is reference to lane that the
           vehicle is generated on."""
        self.on = road #Give the car a pointer to the lane it is on
        road.on.append(self)       
 
        if is_top_lane: lane = road.top_up_lane
        else: lane = road.bottom_down_lane

        lane.on.append(self) #The lane has a list of objects on it

        self.on_road = True #on_road is false if we run off the road

        #For simplicity we assume the car starts at the centre of the road, heading
        # parallel to the course of the road
        self.heading = lane.direction


        x_disp = lane.width/2
        #Randomly place the car somewhere along the length of the road
        y_disp = (self.length/2)+ random.random()*(lane.length-self.length)
        direction = self.heading
        disp = angularToCartesianDisplacement(x_disp,y_disp,direction)
  
        self.y_com = lane.y+disp[1]
        self.x_com = lane.x+disp[0]

        #The coordinates of each corner of the car
        self.getFourCorners()
        
        #NOTE: In the future this should be changed so that intial speed is something
        #      reasonable and not just arbitrarily selected.
        self.v = 5.5 #19.8km/h units are metres per second 


    def move(self,accel,turn_angle):
        """The motion dynamics of the vehicle. Given an input acceleration and wheel-
           angle this determines how much the vehicle should move, and then resets the
           vehicles coordinates appropriately."""
        #The changes induced by the dynamics
        x_dot = self.v*math.cos(math.radians(turn_angle))
        y_dot = self.v*math.sin(math.radians(turn_angle))
        head_dot = turn_angle
        v_dot = accel
        
        #Applying the changes calculated above
        self.y_com += self.timestep*y_dot
        self.x_com += self.timestep*x_dot
        for entry in self.four_corners:
            self.four_corners[entry][0] += self.timestep*x_dot
            self.four_corners[entry][1] += self.timestep*y_dot
        self.v += self.timestep*v_dot
        self.heading += self.timestep*head_dot


    def angularToCartesianDisplacement(x_disp,y_disp,direction):
        phi = math.sqrt(x_disp**2 + y_disp**2)
        delta = math.degrees(math.atan(x_disp/y_disp))

        mod_x,mod_y = 1,1

        #Direction is up/down
        if (direction in range(46,135)) or (direction in range(226,315)):
            if direction in range(46,135): mod_x *= -1
            else: mod_y *= -1
            omega = direction+delta
        #Direction is left/right
        else:
            if direction in range(135,226): mod_x *= -1
            else: mod_y *= -1
            omega = direction-delta

       return (mod_x*phi*math.cos(math.radians(omega)),mod_y*phi*math.sin(math.radians(\
               omega)))


    def checkOnRoad(self):
        on_direct = self.on.direction
        if  
        

    def checkForTransition(self):
         

    def checkPositState(self):
        """Check if the vehicle has crashed into any other vehicle/obstacle/run
           off the road and, if not, check if the vehicle has moved to a new road
           object."""

        #First check if you are moving onto a new road section
        self.checkForTransition()
        self.checkForCrash()
        self.checkOnRoad()
        self.checkForTransition()


    def sense(self):
        """Change the sense variables to match the vehicle's new position/capture
           changes in state."""
        self.checkPositState()


    def printStatus(self,mod=""):
        print("{}{}\tON: {}\tHEAD: {}\tSPEED: {}\tCOM: ({},{})".format(mod,\
               self.label,self.on.label,self.heading,self.v,round(self.x_com,2),\
               round(self.y_com,2)))


def angularToCartesianDisplacement(x_disp,y_disp,direction):
    phi = math.sqrt(x_disp**2 + y_disp**2)
    delta = math.degrees(math.atan(x_disp/y_disp))

    mod_x,mod_y = 1,1

    #Direction is up/down
    if (direction in range(46,135)) or (direction in range(226,315)):
        if direction in range(46,135): mod_x *= -1
        else: mod_y *= -1
        omega = direction+delta
    #Direction is left/right
    else:
        if direction in range(135,226): mod_x *= -1
        else: mod_y *= -1
        omega = direction-delta

   return (mod_x*phi*math.cos(math.radians(omega)),mod_y*phi*math.sin(math.radians(\
           omega)))
