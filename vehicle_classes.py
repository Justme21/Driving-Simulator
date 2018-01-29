import math
import random
import road_classes

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

        #Dictionary storing the coordinates of the four corners of the 
        #vehicle from the orientation of the car
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}
  
        #Velocity of the car
        self.v = None

        #Heading of the car (direction it is facing in degrees)
        self.heading = None

        #Dimensions of the car
        self.length = car_length
        self.width = car_width

        #The object (road or junction) that the car is on
        #This is a list as the car might be straddling a lane and junction
        #Or two lanes
        self.on = []

        #The processing speed of the car (how often it changes it's action)
        self.timestep = timestep
        
        #Sense Variables
        self.on_road = True #Check if car still on road
        self.crashed = False #Check if car has crashed into something

        self.label = "C{}".format(label)
        
        #Initialise the position, velocity and heading features
        self.initSetup(road,is_top_lane)
        

    def setFourCorners(self):
        """Compute the initial coordinates of the four corners of the car (with respect
           to the heading). These values are stored and continuously updated in a 
           dictionary for ease of identifying crashes later on"""
        direction = self.heading
        phi = math.sqrt((self.length/2)**2 + (self.width/2)**2)
        delta = math.degrees(math.atan(self.width/self.length))
        self.four_corners["front_left"] = [self.x_com+phi*math.cos(math.radians(\
                                           direction+delta)),self.y_com-phi*math.sin(\
                                           math.radians(direction+delta))]
        self.four_corners["front_right"] = [self.x_com+phi*math.cos(math.radians(\
                                           direction-delta)),self.y_com-phi*math.sin(\
                                           math.radians(direction-delta))]
        self.four_corners["back_left"] = [self.x_com-phi*math.cos(math.radians(\
                                           direction-delta)),self.y_com+phi*math.sin(\
                                           math.radians(direction-delta))]
        self.four_corners["back_right"] = [self.x_com-phi*math.cos(math.radians(\
                                           direction+delta)),self.y_com+phi*math.sin(\
                                           math.radians(direction+delta))]


    def putOn(self,obj):
        """Want to include 'obj' in the set of objects that 'self' is on or that are on
           'self'. If the object is not already in the list 'self.on' it is addded and,
           symmetrically, if self is not in the object's list it is added"""
        if obj not in self.on:
            self.on.append(obj)
        if self not in obj.on:
            obj.putOn(self)


    def takeOff(self,obj):
        """Remove object 'obj' from 'self's list of objects it is on or that are on it.
           First remove the object from the list if it is on it, then check if self is
           still on the object's list, in which case remove it."""
        if obj in self.on:
            self.on.remove(obj)
        if self in obj.on:
            obj.takeOff(self)


    def initSetup(self,road,is_top_lane):
        """Performing initial setup of the car. Input is reference to lane that the
           vehicle is generated on."""
        lane = None
        if is_top_lane:
            self.putOn(road.top_up_lane)
            lane = road.top_up_lane
        else:
            self.putOn(road.bottom_down_lane)
            lane = road.bottom_down_lane

        self.on_road = True #on_road is false if we run off the road

        #For simplicity we assume the car starts at the centre of the road, heading
        # parallel to the course of the road
        self.heading = lane.direction

        x_disp = round(lane.width/2,2)
        #Randomly place the car somewhere along the length of the road
        y_disp = round((self.length/2)+ random.random()*(lane.length-self.length),2)
        direction = self.heading
        disp = angularToCartesianDisplacement(x_disp,y_disp,direction)
  
        lane_coords = lane.four_corners["front_left"]
        lane_x = lane_coords[0]
        lane_y = lane_coords[1]
        self.y_com = round(lane_y+disp[1],2)
        self.x_com = round(lane_x+disp[0],2)

        print("TEST BEGIN")
        print("LANE: ({},{})\tDIREC: {}\tCAR: ({},{})\tDISP: ({},{}))".format(\
               lane_x,lane_y,direction,self.x_com,self.y_com,disp[0],disp[1]))
        print("TEST END")

        #The coordinates of each corner of the car
        self.setFourCorners()
        
        #NOTE: In the future this should be changed so that intial speed is something
        #      reasonable and not just arbitrarily selected.
        self.v = 5.5 #19.8km/h units are metres per second 


    def move(self,accel,turn_angle):
        """The motion dynamics of the vehicle. Given an input acceleration and wheel-
           angle this determines how much the vehicle should move, and then resets the
           vehicles coordinates appropriately."""
        #The changes induced by the dynamics
        x_dot = self.v*math.cos(math.radians(self.heading))
        y_dot = self.v*math.sin(math.radians(self.heading))
        #x_dot = self.v*math.cos(math.radians(turn_angle))
        #y_dot = self.v*math.sin(math.radians(turn_angle))
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


    def checkNewOn(self):
        candidates = []
        dist = None
        obj_coords = None
        front,back = False,False
        coord = self.four_corners
        for obj in list(self.on):
            obj_coord = obj.four_corners
            if isinstance(obj,road_classes.Lane):
                #Find the point that is possibly inside a junction
                #The furthest a point could be into a junction is when angle is 45 degrees
                front_pt = [(obj_coord["front_left"][i] + \
                            obj_coord["front_right"][i])/2 for i in range(2)]
                back_pt = [(obj_coord["back_left"][i] + \
                            obj_coord["back_right"][i])/2 for i in range(2)]
                if computeDistance((self.x_com,self.y_com),front_pt) < \
                    obj.width+self.length/2 : 
                    candidates.append(obj.to_junction)
                if computeDistance((self.x_com,self.y_com),back_pt) < \
                    obj.width+self.length/2 : 
                    candidates.append(obj.from_junction)

            if isinstance(obj,road_classes.Junction):
                for lane in obj.in_lanes:
                    if math.fabs(((lane.direction+180)%360)-self.direction)<45:
                        candidates.append(lane)
                for lane in obj.out_lanes:
                    if math.fabs(lane.direction-self.direction)<45:
                        candidates.append(lane)

        for entry in candidates:
            if checkOn(self,entry):
                self.putOn(entry)


    def checkNewOff(self):
        left_right = None
        top_bottom = None
        for obj in self.on:
            print("Checking if off {}".format(obj.label))
            if not checkOn(self,obj):
                self.takeOff(obj)


    def checkForCrash(self):
        """Determines whether or not the car has crashed into another object.
           Returns True/False and a list containing all the objects it has crashed into
           (empty if it has not crashed)"""
        has_crashed = False
        potentials,crashed = [],[]
        for obj in self.on:
            for entry in obj.on:
                if entry not in potentials and entry is not self:
                    potentials.append(entry)

        for veh in potentials:
            veh_coords = veh.four_corners
            for entry in self.four_corners:
                pt = self.four_corners[entry]
                if sideOfLine(pt,veh_coords["front_left"],coords["front_right"])!=\
                   sideOfLine(pt,veh_coords["back_left"],coords["back_right"]):
                       has_crashed = True
                       crashed.append(veh)
                       break
                if sideOfLine(pt,veh_coords["front_left"],coords["back_left"])!=\
                   sideOfLine(pt,veh_coords["front_right"],coords["back_right"]):
                       has_crashed = True
                       crashed.append(veh)
                       break
        return has_crashed,crashed


    def checkPositState(self):
        """Check if the vehicle has crashed into any other vehicle/obstacle/run
           off the road and, if not, check if the vehicle has moved to a new road
           object."""

        #First check if you are moving onto a new road section
        self.checkNewOn()
        print("Done Checking On")
        self.checkNewOff()
        print("Done Checking Off")
        crashed,crash_list = self.checkForCrash()


    def sense(self):
        """Change the sense variables to match the vehicle's new position/capture
           changes in state."""
        self.checkPositState()


    def printStatus(self,mod=""):
        corner_labels = {"back_right":"br","back_left":"bl","front_right":"fr",\
                         "front_left":"fl"}
        dims = ""
        for x in self.four_corners:
            dims += "{}({},{}), ".format(corner_labels[x],round(self.four_corners[x][0],2),\
                                         round(self.four_corners[x][1],2))
        print("{}{}\tON: {}\tHEAD: {}\tSPEED: {}\n{}\t {}".format(mod,\
               self.label,[x.label for x in self.on],self.heading,self.v,\
               self.label,dims))


def checkOn(car, obj):
    """Returns true if the car is sufficiently on the object, and false otherwise.
       'Sufficiently' means in at least one dimension the vehicle is entirely within
       the boundary of the object, and in the other there are at least two wheels on"""
    left_right = 0
    top_bottom = 0
    coords = obj.four_corners
    for entry in car.four_corners:
        pt = car.four_corners[entry]
        if sideOfLine(pt,coords["front_left"],coords["front_right"]) !=\
           sideOfLine(pt,coords["back_left"],coords["back_right"]):
           top_bottom += 1
        if sideOfLine(pt,coords["front_left"],coords["back_left"]) !=\
           sideOfLine(pt,coords["front_right"],coords["back_right"]):
           left_right += 1


    print("{}: TB:{}\t LR:{}".format(obj.label,top_bottom,left_right))
    car.printStatus()
    obj.printStatus()
    exit(-1)
    if max(top_bottom,left_right)==4 and min(top_bottom,left_right)>=2:
        return True
    else:
        return False


def computeDistance(pt1,pt2):
    return math.sqrt((pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2)


def sideOfLine(pt,line_strt,line_end):
    if line_strt[0] == line_end[0]:
        if pt[0]>line_strt[0]: return -1
        else: return 1
    else:
        m = (line_strt[1]-line_end[1])/(line_strt[0]-line_end[0])
    y_test = line_strt[1] + m*(line_strt[0]-pt[0])
    if pt[1]>y_test: return 1
    elif pt[1]<y_test: return -1
    else: return 0


def angularToCartesianDisplacement(x_disp,y_disp,direction):
    phi = math.sqrt(x_disp**2 + y_disp**2)
    delta = math.degrees(math.atan(x_disp/y_disp))
    omega = direction + delta

    return (-phi*math.cos(math.radians(omega)),phi*math.sin(math.radians(\
           omega)))
