import controller_classes
import math
import random
import road_classes

#Dimensions are those of a "Ford Focus 5-door 1.8i Zetec" as found at
# http://www.metric.org.uk/motoring  . Units are metres
car_length = 4.17
car_width = 1.7

class Car():
    def __init__(self,road,is_top_lane,v,is_ego,label,debug,controller=None,timestep=.1):
        #x and y of the centre of mass (COM) of the car.
        # For simplicity we assume the COM is the centre of the car
        self.x_com = None
        self.y_com = None

        #Dictionary storing the coordinates of the four corners of the 
        #vehicle from the orientation of the car
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}

        #Velocity and accelerations of the car of the car
        self.v = None
        self.accel = 0
        self.angle_accel = 0

        #Heading of the car (direction it is facing in degrees)
        self.heading = None

        #The label of the junction that the car is to drive to
        self.is_ego = is_ego

        #Dimensions of the car
        self.length = car_length
        self.width = car_width

        #The object (road or junction) that the car is on
        #This is a list as the car might be straddling a lane and junction
        #Or two lanes
        self.on = []

        #The processing speed of the car (how often it changes it's action)
        self.timestep = timestep
        self.debug = debug

        #Sense Variables
        self.on_road = True #Check if car still on road
        self.crashed = False #Check if car has crashed into something
        self.is_complete = False

        self.label = "C{}".format(label)

        #Initialise Trajectory and Waypoint
        self.traj_posit = 0
        self.trajectory,self.waypoints = initialiseTrajectory(road,is_top_lane)
        if self.debug:
            print("{} has TRAJECTORY: {}".format(self.label,[x.label for x in self.traj]))

        #Initialise Public and Private State
        self.pub_state = None
        self.priv_state = None

        self.repo_len = int((1/timestep)*2) #Store the past 2 seconds of state information
        self.public_state_repo = []

        #Initialise the position, velocity and heading features
        self.initSetup(road,v,is_top_lane)

        #Initilaising the Controller
        if controller is not None:
            self.controller = controller
        else:
            self.controller = controller_classes.randomController()

        #Initialise time. Used to record how long it takes to achieve an objective
        self.time = 0


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


    def initSetup(self,road,v,is_top_lane):
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

        #NOTE" DEBUGGING
        if self.debug:
            print("TEST BEGIN")
            print("LANE: ({},{})\tDIREC: {}\tCAR: ({},{})\tDISP: ({},{}))".format(\
                   lane_x,lane_y,direction,self.x_com,self.y_com,disp[0],disp[1]))
            print("TEST END")

        #The coordinates of each corner of the car
        self.setFourCorners()

        #NOTE: In the future this should be changed so that intial speed is something
        #      reasonable and not just arbitrarily selected.
        self.v = v #19.8km/h units are metres per second 


    def composeState():
        return self.pub_state+self.priv_state


    def chooseAction(self):
        state = self.composeState()
        accel_cat,angle_cat = self.controller.selectAction(state)

    def move(self):
        """The motion dynamics of the vehicle. Given an input acceleration and wheel-
           angle this determines how much the vehicle should move, and then resets the
           vehicles coordinates appropriately."""

        accel,turn_angle = self.chooseAction()
        #The changes induced by the dynamics
        x_dot = self.v*math.cos(math.radians(self.heading))
        #y_dot is set as negative to reflect the fact the pygame coordinate space
        y_dot = -self.v*math.sin(math.radians(self.heading))
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
        """Check to see if self is now on any of the environment objects that it 
           was not on in the previous round."""
        #The list of objects we need to test to check if we are on.
        candidates = []
        dist = None
        obj_coords = None
        coord = self.four_corners
        #We know the only new things we might be on must be linked to things
        # we were already on. Thus we find candidates by considering self.on.
        for obj in list(self.on):
            obj_coord = obj.four_corners

            if isinstance(obj,road_classes.Lane):
                #We compute the distance from the car to the object by choosing
                #The midpoint of each end of the object (easiest).
                front_pt = [(obj_coord["front_left"][i] + \
                            obj_coord["front_right"][i])/2 for i in range(2)]
                back_pt = [(obj_coord["back_left"][i] + \
                            obj_coord["back_right"][i])/2 for i in range(2)]
                #If front of car is within lane_width of front/back of lane
                #Then it is worth checking if we have transitioned onto the
                #next/previous junction for the lane.
                if computeDistance((self.x_com,self.y_com),front_pt) < \
                    obj.width+self.length/2 and obj.to_junction not in self.on: 
                    candidates.append(obj.to_junction)
                if computeDistance((self.x_com,self.y_com),back_pt) < \
                    obj.width+self.length/2 and obj.from_junction not in self.on: 
                    candidates.append(obj.from_junction)

                #It is always worth checking to make sure we have not drifted
                #onto opposite lane.
                if obj.lane_twin not in self.on:
                    candidates.append(obj.lane_twin)

            if isinstance(obj,road_classes.Junction):
                #If we are on a junction we only check to see if we are on a 
                # lane whose angle is sufficiently similar to our own (within 45 degrees).
                # It might be better to test for distance, but it feels unnecessary.
                if self.debug: print("Testing Candidates in Junction {}".format(obj.label))
                for lane in obj.in_lanes:
                    if self.debug:
                        print("IN: {}\t HEADING {}\tLANE DIRECTION {} ({})".format(obj.label,\
                            self.heading,lane.direction,(lane.direction+180)%360))
                    if math.fabs(((lane.direction+180)%360)-self.heading)<45:
                        candidates.append(lane)
                for lane in obj.out_lanes:
                    if self.debug:
                        print("OUT: {}\t HEADING {}\tLANE DIRECTION {} ({})".format(obj.label,\
                               self.heading,(lane.direction+180)%360,lane.direction))
                    if math.fabs(lane.direction-self.heading)<45:
                        candidates.append(lane)

        return candidates


    def testCandidates(self,candidates):
        """Given a list of candidates it tests to see if self in on any of them.
           If there is evidence that self is on a candidate it is put on self.on"""
        for entry in candidates:
            if self.debug:
                print("Testing Candidate {}".format(entry.label))
            if checkOn(self,entry):
                self.putOn(entry)


    def checkNewOff(self):
        """Checks if there is evidence that self has left any of the objects
           in self.on"""
        left_right = None
        top_bottom = None
        for obj in list(self.on):
            if self.debug:
                print("Checking if off {}".format(obj.label))
            if not checkOn(self,obj):
                self.takeOff(obj)


    def checkForCrash(self):
        """Determines whether or not the car has crashed into another object.
           Returns True/False and a list containing all the objects it has crashed into
           (empty if it has not crashed)"""
        has_crashed = False
        potentials,crashed = [],[]
        #We want to consider all the things that are on everything that we are on
        # Since Roads are not stored in the list of things we are on this will only
        # be other cars
        for obj in self.on:
            for entry in obj.on:
                if entry not in potentials and entry is not self:
                    potentials.append(entry)

        for veh in potentials:
            veh_coords = veh.four_corners
            for entry in self.four_corners:
                pt = self.four_corners[entry]
                if sideOfLine(pt,veh_coords["front_left"],veh_coords["front_right"])!=\
                   sideOfLine(pt,veh_coords["back_left"],veh_coords["back_right"]) and\
                   sideOfLine(pt,veh_coords["front_left"],veh_coords["back_left"])!=\
                   sideOfLine(pt,veh_coords["front_right"],veh_coords["back_right"]):
                       has_crashed = True
                       crashed.append(veh)
                       break
        return has_crashed,crashed


    def checkPositState(self):
        """Check if the vehicle has crashed into any other vehicle/obstacle/run
           off the road and, if not, check if the vehicle has moved to a new road
           object."""

        #First check if you are moving onto a new road section
        if self.debug: print("Checking On Anything New")
        on_candidates = self.checkNewOn()
        if self.debug: print("Done Checking On")
        #Check if you have left anything you were on before
        self.checkNewOff()
        #Check if we are sufficiently "on" any of the candidates identified earlier
        self.testCandidates(on_candidates)
        if self.debug: print("Done Checking Off")

        #Checking if we have wandered off the map
        self.on_road = False
        for entry in self.on:
            if isinstance(entry,road_classes.Lane) or isinstance(entry,road_classes.Junction):
                self.on_road = True
                break

        #No point repeatedly checking for a crash that already happened
        if not self.crashed:
            crashed,crash_list = self.checkForCrash()
            if crashed:
                self.crashed = True
                for entry in crash_list: entry.crashed = True
                #Think of a better way to address crashing here.
                if self.debug:
                    print("Oh MY GOSH A CRASH!")
                    self.printStatus()
                    for entry in crash_list:
                        entry.printStatus()
                    exit(-1)

        #Progress the Trajectory Goal
        on = None
        for entry in self.on:
            on = entry
            if isinstance(on,road_classes.Lane):
                on = on.road
            if on is self.trajectory(self.traj_posit):
                self.traj_posit += 1
                break
        if self.traj_posit == len(self.trajectory): self.is_complete = True
        if self.is_complete and not self.is_ego:
            next_lane = random.choice(on.out_lanes)
            self.trajectory,self.waypoints = initialiseTrajectory(next_lane,next_lane.is_top_up)
            self.is_complete = False


    def updatePublicState(self):
        state = []
        #Position of the vehicle. Mainly for relative computation for other cars
        #NOTE: Come back to this later. It seems clear that this will not be useful in computing
        # the ego state.
        #state += [self.x_com,self.y_com]

        #Velocity parallel and perpendicular to the current road
        alpha = self.heading
        i = 0
        while i<len(self.on) and not (isinstance(self.on[i],road_classes.Lane) or\
                   isinstance(self.on[i],road_classes.Junction)): i+=1
        beta = self.on[i].direction
        v_par = self.v*math.cos(math.radians(alpha-beta))
        v_perp = self.v*math.sin(math.radians(alpha-beta))
        state += [v_par,v_perp]

        #Heading
        #state.append(self.heading)
        state.append(beta-alpha)

        self.pub_state = state

        while len(self.public_state_repo)>=self.repo_len: del(self.public_state_repo[0])
        self.public_state_repo.append(state)


    def updatePrivateState(self):
        state = []
        #The time the vehicle has been runnning on this trajectory for
        state.append(self.time)
        #The distance from the next waypoint in the trajectory
        waypt = self.waypoints[self.traj_posit]
        ego_com = (self.x_com,self.y_com)
        state.append(computeDistance(ego_com,waypt))

        #The perpendicular distance to either side of the lane/junction
        obj = self.trajectory[self.traj_posit]
        if obj.direction%180 == 90:
            state.append(math.fabs(ego_com[0]-obj.four_corners["back_right"][0]))
            state.append(math.fabs(ego_com[0]-obj.four_corners["back_left"][0]))
        else:
            slope = math.tan(obj.heading)
            coord_right = obj.four_corners["back_right"]
            coord_left = obj.four_corners["back_left"]
            #Formula for perpendicular distance between point and a line in form Ax+By+C
            # where A = -slope, B=1 and C=(slope*x1-y1)
            # d = |Ax+By+c|/<sqrt>(A^2+B^2)
            state.append(math.fabs(-slope*ego_com[0]+ego_com[1]+(slope*coord_right[0]-coord_right[1]))/\
                    math.sqrt(slope**2+1))
            state.append(math.fabs(-slope*ego_com[0]+ego_com[1]+(slope*coord_left[0]-coord_left[1]))/\
                    math.sqrt(slope**2+1))

        #The current acceleration and angular acceleration
        state.append(self.accel)
        state.append(self.angle_accel)
        #Whether or not the car has crashed and whether or not it is still on the road
        state.append(self.crashed)
        state.append(self.on_road)
        state.append(self.is_complete)


    def sense(self):
        """Change the sense variables to match the vehicle's new position/capture
           changes in state."""
        #Perform checks for some of the private state values
        self.checkPositState()
        #Update the Public State values
        self.updatePublicState()
        #Update the Private state values
        self.updatePrivateState()


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


def computeTrajectoryPoint(stop_point,front_back_label):
    fl = stop_point.four_corners["{}_left".format(front_back_label)]
    fr = stop_point.four_corners["{}_right".format(front_back_label)]
    pt = ((fl[0] + fr[0])/2,(fl[1]+fr[1])/2)
    return pt


def buildTrajectory(start_point):
    next_stop = start_point.to_junction #next_stop is junction next to be entered
    trajectory = []
    waypoints = []
    pt = computeTrajectoryPoint(start_point,"front")
    trajectory.append(next_stop)
    waypoints.append(pt)
    #Trajectories must be at least 3 stops long but no more than 9
    while len(trajectory)<10 and (len(trajectory)<3 or random.random()<.5):
        #Choosing a lane out of the junction to add to the trajectory
        while next_stop in trajectory:
            next_stop = random.choice(next_stop.out_lanes)
        pt = computeTrajectoryPoint(next_stop,"back")
        trajectory.append(next_stop.road) #Here next_stop is the lane that leads out of the junction
        waypoints.append(pt)

        #Adding the junction the current lane leads into
        pt = computeTrajectoryPoint(next_stop,"front")
        next_stop = next_stop.to_junction
        trajectory.append(next_stop)
        waypoints.append(pt)
        #Here we are allowing loops to form in the trajectory. But they end the trajectory
        #if the current junction was previously in the trajectory then it is a terminal point
        if next_stop in trajectory[:-1]: break
    return trajectory,waypoints


def initialiseTrajectory(road,is_top_lane):
    """Given the current road the vehicle is on (and boolean indicating lane on the road)
       intialises and returns a new trajectory with the next point the end of that lane"""
    lane = None
    if is_top_lane:
        lane = road.top_up_lane
    else:
        lane = road.bottom_down_lane
    traj,waypoints = buildTrajectory(lane)
    return traj,waypoints


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


    if car.debug:
        print("{}: TB:{}\t LR:{}".format(obj.label,top_bottom,left_right))
        car.printStatus()
        obj.printStatus()
        print("\n")
    if max(top_bottom,left_right)==4 and min(top_bottom,left_right)>=2:
        return True
    else:
        return False


def computeDistance(pt1,pt2):
    """Compute the L2 distance between two points"""
    return math.sqrt((pt2[1]-pt1[1])**2 + (pt2[0]-pt1[0])**2)


def sideOfLine(pt,line_strt,line_end):
    """Determine which side of the line segment [line_strt,line_end] the point
       pt lies on."""
    #Slope here is infinite so side of the line comes down to the x-values
    if line_strt[0] == line_end[0]:
        if pt[0]>line_strt[0]: return -1
        else: return 1
    else:
        m = (line_strt[1]-line_end[1])/(line_strt[0]-line_end[0])
    y_test = line_strt[1] + m*(pt[0]-line_strt[0])
    if pt[1]>y_test: return 1
    elif pt[1]<y_test: return -1
    else: return 0


def angularToCartesianDisplacement(x_disp,y_disp,direction):
    """Translates a displacement in a specified direction into displacement along the 
       standard basis axes"""
    phi = math.sqrt(x_disp**2 + y_disp**2)
    delta = math.degrees(math.atan(x_disp/y_disp))
    omega = direction + delta

    return (-phi*math.cos(math.radians(omega)),phi*math.sin(math.radians(\
           omega)))
