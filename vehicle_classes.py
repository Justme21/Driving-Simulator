# -*- coding: utf-8 -*-
import math
import random
import road_classes
import trajectory_builder

#Values corresponding to a Volvo S60 (values found at https://www.auto123.com/en/new-cars/technical-specs/volvo/s60/2018/base/t5-base-awd/#dimensions)
#Same vehicle used in "model-based threat assessment for avoiding arbitrary vehicle collisions"-Br\{"}annstr\{"}om
CAR_LENGTH = 4.635
CAR_WIDTH = 2.097

#Alternative values could be length=5m, width=2 as per "model-based threat assessment for avoiding arbitrary vehicle collisions"-Br\{"}annstr\{"}om

class Car():
    def __init__(self,controller=None,is_ego=False,label="DEFAULT_LABEL",debug=False,is_demo=True,is_interactive=True,timestep=.1,car_params={}):
        ########################### State Variables ###########################################
        #x and y of the centre of mass (COM) of the car.
        # For simplicity we assume the COM is the centre of the car
        self.x_com = None
        self.y_com = None

        #Dictionary storing the coordinates of the four corners of the 
        #vehicle from the orientation of the car
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}

        #Velocity and accelerations of the car of the car
        self.v = None #units are metres per second
        self.accel = None # metres per second squared
        self.yaw_rate= None #radians/degrees per metres squared (?)

        #Heading of the car (direction it is facing in degrees)
        self.heading = None

        #The label of the junction that the car is to drive to
        self.is_ego = is_ego

        ########################## Dynamics Attributes #######################################
        #Dimensions of the car
        if "length" in car_params and car_params["length"]>0: self.length = car_params["length"]
        else: self.length = CAR_LENGTH

        if "width" in car_params and car_params["width"]>0: self.width = car_params["width"]
        else: self.width = CAR_WIDTH


        #Kinematic Bicycle model (The Kinematic Bicycle Model: a Consistent Model for Planning Feasible Trajectories for Autonomous Vehicles?)
        #Philip Polack, Florent Altché, Brigitte D’Andréa-Novel, Arnaud De La Fortelle
        if "com_to_axle" in car_params and (max(car_params["com_to_axle"])>0):
            self.Lr = car_params["com_to_axle"][0]
            self.Lf = car_params["com_to_axle"][1]
        else:
            self.Lr = 1.61 #Distance from COM to rear axle
            self.Lf = 1.11 #Distance from COM to front axle

        #The processing speed of the car (how often it changes it's action)
        self.timestep = timestep #unit is seconds

        ####################33 Relating Car to Map ####################### 
        #The object (road or junction) that the car is on
        #This is a list as the car might be straddling a lane and junction
        #Or two lanes
        self.on = []

        #Sense Variables
        self.on_road = True #Check if car still on road
        self.crashed = False #Check if car has crashed into something
        self.is_complete = False
        self.in_demo = is_demo #if car is in demo mode then all non-ego cars cycle through objectives (see is_complete). 
                               #When not in demo is_complete for non-ego is not reset
        self.is_interactive = is_interactive #if car is not interactive then it cannot crash into other vehicles or be crashed into
                                             # this is useful if you want to duplicate a car to generate data but don't want the original car to crash into duplicate 

        ####################### Trajectory Stuff #######################
        #Trajectory and Waypoint
        self.destination = None #Road object that is what the car is (theoretically) going towards
        self.traj_posit = None #Index giving position along the trajectory
        self.trajectory = None #List of junctions that connect current lane to the destination
        self.waypoints = None #Coordinates of entries in trajectory
        self.waypoint_distance = None

        #Initialise Public and Private State
        self.pub_state = None
        self.priv_state = None

        ############# Controllers Stuff ################################
        self.controller = controller
        if self.controller is not None: self.controller.setup(ego=self)
        #Keeps track of permissible controllers for a vehicle
        self.controllers = {"default": controller}

        #Triggers; keys of dictionary is a binary function. At the end of each round of simulation (in endStep)
        # If the key returns true then the corresponding function is executed.
        self.triggers = {}

        ############# Final Touches ###################################
        #Initialise time. Used to record how long it takes to achieve an objective
        self.time = 0
        self.debug = debug
        self.label = "C{}".format(label) #Car Label
        self.initialisation_params = {}


    #property tag means this function can be called as an attribute (veh.state will be the state
    # returned by composeState
    @property
    def state(self):
        """Define the state of the car so that it can be called at any time during the simulation"""
        return self.composeState()


    def copy(self,dup=None):
        """Returns a car object with the same initial parameters as the car being copied.
           The copy is,by default, not interactive since, if it were interactive, it would immediately crash into the car it was copied from"""
        if dup is None:
            dup = Car(controller=None,label="DUMMY{}".format(self.label),is_ego=self.is_ego,is_demo=self.in_demo,is_interactive=False,timestep=self.timestep)
        dup.controllers = {}
        if self.debug:
            print("Loading controllers from {} to {}".format(self.label,dup.label))
        for controller in self.controllers:
            if self.controllers[controller] is not None:
                dup.controllers[controller] = self.controllers[controller].copy(target=dup)
            else:
                dup.controllers[controller] = None
            if self.controller == self.controllers[controller]:
                #Not sure what the initial setting for the controller should be
                #Initially I thought it should be default, but this creates problems if vehicle
                # has different controllers at different times
                dup.controller = dup.controllers[controller]
        if self.debug:
            print("Finished loading controllers from {} to {}".format(self.label,dup.label))
        if self.initialisation_params != {}:
            #If this car has already been initialised then copy should also be initialsed
            #This means if we reset this vehicle it will reset to the same place as the original  
            dup.initSetup(**self.initialisation_params)

            #Set the current state of the duplicate to match the original
            dup.setMotionParams((self.x_com,self.y_com),self.heading,self.v)
            dup.setAction(self.accel,self.yaw_rate)
            for obj in self.on:
                #What objects you can collide with/be on depends on what you are currently on
                # so manually put on the objects the original is currently on.
                dup.putOn(obj)
            #Update the duplicate's state
            dup.sense()
        return dup


    def distToObj(self):
        """Returns distance from objective in terms of number of locations
           self.trajectory is ordered list of junctions and roads you need to
           pass to reach objective.
           self.traj_posit is the index of the trajectory you are currently at"""
        return len(self.trajectory)-self.traj_posit


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


    def initSetup(self,road_lane,dest,v,heading=None,accel=0,yaw_rate=0,prev_disp_x=None,prev_disp_y=None):
        """Performing initial setup of the car. Input is reference to lane that the
           vehicle is generated on."""
        self.on = []
        lane = None
        road = road_lane[0]
        #self.putOn(road) #controversial. Originally road was only a wrapper for lanes, but tentatively changing this
        is_top_lane = bool(road_lane[1])
        if is_top_lane:
            self.putOn(road.top_up_lane)
            lane = road.top_up_lane
        else:
            self.putOn(road.bottom_down_lane)
            lane = road.bottom_down_lane

        #For simplicity we assume the car starts at the centre of the road, heading
        # parallel to the course of the road (unless heading specified) (heading only specified for copies)
        if heading is None:
            heading = lane.direction
        self.heading = heading

        self.on_road = True #on_road is false if we run off the road
        self.crashed = False
        self.is_complete = False

        if prev_disp_x is None:
            x_disp = round(lane.width/2,2)
        else: x_disp = prev_disp_x

        if prev_disp_y is None:
            #Randomly place the car somewhere along the length of the road
            y_disp = round((self.length/2)+ random.random()*(lane.length-self.length),2)
        else:
            y_disp = prev_disp_y

        #disp = angularToCartesianDisplacement(x_disp,y_disp,heading)
        disp = angularToCartesianDisplacement(x_disp,y_disp,lane.direction)

        lane_coords = lane.four_corners["front_left"]
        lane_x = lane_coords[0]
        lane_y = lane_coords[1]
        self.y_com = round(lane_y+disp[1],2)
        self.x_com = round(lane_x+disp[0],2)

        #NOTE" DEBUGGING
        if self.debug:
            print("TEST BEGIN")
            print("LANE: ({},{})\tDIREC: {}\tCAR: ({},{})\tDISP: ({},{}))".format(\
                   lane_x,lane_y,heading,self.x_com,self.y_com,disp[0],disp[1]))
            print("TEST END")

        #The coordinates of each corner of the car
        self.setFourCorners()

        self.v = v

        self.accel = accel
        self.yaw_rate = yaw_rate

        #Initialise Trajectory and Waypoint
        #dest also contains [0,1] lane specification, but for the time being we don't use that
        if dest[1]:
            self.destination = dest[0].top_up_lane.to_junction
        else:
            self.destination = dest[0].bottom_down_lane.to_junction
        self.traj_posit = 0
        self.trajectory,self.waypoints = trajectory_builder.initialiseTrajectory(road,is_top_lane,self.destination)
        if self.debug:
            print("{}: Trajectory is built".format(self.label))
            print("{} has TRAJECTORY: {}".format(self.label,[x.label for x in self.trajectory]))

        self.sense()

        self.initialisation_params = {"road_lane":road_lane,"dest":dest,"v":v,"heading":heading,"accel":accel,"yaw_rate":yaw_rate,"prev_disp_x":x_disp,"prev_disp_y":y_disp}


    def reinitialise(self):
        params = self.initialisation_params
        self.time = 0
        self.initSetup(**params)
        for controller in self.controllers.values():
            controller.reset()
        self.controller = self.controllers["default"]


    def composeState(self):
        """Merges the public and private states into a single dictionary leaving both states unchanged"""
        pass_state = {}
        pass_state.update(self.pub_state)
        pass_state.update(self.priv_state)
        return pass_state


    def chooseAction(self,accel_range=None,yaw_rate_range=None,particles=None):
        """Merge the public and private states and pass this to the vehicle controller to get linear and angular accelerations"""
        state = self.composeState()
        if accel_range is None: accel_range = [None,None]
        if yaw_rate_range is None: yaw_rate_range = [None,None]

        if particles is None:
            self.accel,self.yaw_rate = self.controller.selectAction(state,accel_range,yaw_rate_range)
        else:
            self.accel,self.yaw_rate = self.controller.selectAction(state,accel_range,yaw_rate_range,particles)

        self.updatePublicState()


    def setAction(self,accel=None,yaw_rate=None):
        if accel is not None:
            self.accel = accel
        if yaw_rate is not None:
            self.yaw_rate = yaw_rate

        self.updatePublicState()


    def setMotionParams(self,posit=None,heading=None,vel=None):
        if posit is not None:
            self.x_com = posit[0]
            self.y_com = posit[1]
        if heading is not None:
            self.heading = heading
        if vel is not None:
            self.v = vel

        if posit is not None or heading is not None: self.setFourCorners()

        self.sense() #This is sense as the change in position might have caused a collision


    def addControllers(self,controller_dict):
        self.controllers.update(controller_dict)


    def setController(self,tag=None,controller=None):
        if controller is not None:
            if controller not in [self.controllers[x] for x in self.controllers]:
                print("Vehicle_Classes Error: Controller can't be set as this controller has not been added to the vehicle")
                print("Controller: {}\tLoaded Controllers: {}".format(controller,self.controllers))
                exit(-1) #Should this be an exit?
            else:
                self.controller = controller
        elif tag is not None:
            if tag not in self.controllers:
                print("Vehicle_Classes Error: Controller can't be set as this controller has not been added to the vehicle")
                print("Controller: {}\tLoaded Controllers: {}".format(tag,self.controllers))
                exit(-1)
            else:
                self.controller = self.controllers[tag]
        else:
            print("Error: No controller specified")
            exit(-1)


    def addTriggers(self,trigger_dict):
        self.triggers.update(trigger_dict)


    def simulateDynamics(self,lin_accel,yaw_rate,init_vel=None,init_heading=None,dt=None):
        """Simulate the consequences of applying linear acceleration v_dot and angular acceleration
           head_dot on the current state of the car using the specified vehicle dynamics"""

        if dt is None:
            #dt is None if the dynamics are being simulated in standard time (as opposed to some specified timestep size
            # that is not the default vehicle timestep. Otherwise just use default
            dt = self.timestep

        slip_angle = math.atan(math.tan(math.radians(yaw_rate))*self.Lr/(self.Lr+self.Lf))

        #IS THIS A CORRECT WAY TO GO FROM rad/s -> deg/s ?
        heading_dot = math.degrees((self.v/self.Lr)*math.sin(slip_angle))
        v_dot = lin_accel

        #It is a bit wasteful to do this computation here and then again when called in "move"
        # but by not having simulate Dynamics actually affect the agent it means other programs
        # can use it

        if init_vel is not None: vel = init_vel
        else: vel = self.v

        if init_heading is not None: head = init_heading
        else: head = self.heading

        #Assuming actions immediately take effect
        v = vel + v_dot*dt
        heading = head + heading_dot*dt
        
        #Delayed repsonse (more common)
        #v = vel
        #heading = head


        x_dot = v*math.cos(math.radians(heading)+slip_angle)
        y_dot = -v*math.sin(math.radians(heading)+slip_angle)

        return x_dot,y_dot,v_dot,heading_dot


    def move(self,dt=None,num_timesteps=1):
        """The motion dynamics of the vehicle. Given an input acceleration and wheel-
           angle this determines how much the vehicle should move, and then resets the
           vehicles coordinates appropriately."""

        if dt is None:
            #if dt is not None then the calling function has specified a timestep size.
            # otherwise just use default. 
            dt = self.timestep

        num_timesteps = max(1,int(num_timesteps)) #Ensure the number of timesteps is always an integer

        x_dot,y_dot,v_dot,heading_dot = self.simulateDynamics(self.accel,self.yaw_rate,dt=dt)


        self.x_com += dt*num_timesteps*x_dot
        self.y_com += dt*num_timesteps*y_dot

        self.v += v_dot*dt*num_timesteps
        self.heading = (self.heading + heading_dot*dt*num_timesteps)%360

        self.time += dt*num_timesteps

        self.setFourCorners()
        self.updatePublicState()


    def rollout(self,action_sequence):
        """Roll out a sequence of actions (acceleration,yaw_rate pairs). The vehicle ends up
           where it would be at the end of the trajectory without interacting with the environment
           in the interim (i.e. passes through cars and crosses lanes etc.)"""
        for (lin_accel,yaw_rate) in action_sequence:
            self.setAction(lin_accel,yaw_rate)
            self.move()

        self.sense()


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

            if isinstance(obj,road_classes.Road):
                for lane in [obj.top_up_lane,obj.bottom_down_lane]:
                    if lane not in self.on:
                        candidates.append(lane)

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

                if obj.to_junction not in self.on and computeDistance((self.x_com,self.y_com),\
                        front_pt) < obj.width+self.length/2:
                    candidates.append(obj.to_junction)
                if obj.from_junction not in self.on and computeDistance((self.x_com,self.y_com),\
                        back_pt) < obj.width+self.length/2:
                    candidates.append(obj.from_junction)

                #It is always worth checking to make sure we have not drifted
                #onto opposite lane.
                if obj.lane_twin not in self.on:
                    candidates.append(obj.lane_twin)

                #I am uncertain about adding the road here as it should be redundant as either
                # the car will be on one of the lanes, in which case it will be automatically
                # put on the road, or else it shouldn't be on the road. But there is an occasinal
                # issue where the car falls between the cracks due to how "checkOn" is defined.
                # This should resolve.
                if obj.road not in self.on:
                    candidates.append(obj.road)

            if isinstance(obj,road_classes.Junction):
                #If we are on a junction we only check to see if we are on a 
                # lane whose angle is sufficiently similar to our own (within 45 degrees).
                # It might be better to test for distance, but it feels unnecessary.
                if self.debug: print("Testing Candidates in Junction {}".format(obj.label))
                for lane in obj.in_lanes:
                    if self.debug:
                        print("IN: {}\t HEADING {}\tLANE DIRECTION {} ({})".format(lane.label,\
                            self.heading,lane.direction,(lane.direction+180)%360))
                    if math.fabs(((lane.direction+180)%360)-self.heading)<45 or math.fabs(((lane.direction+180)%360)-self.heading)>315:
                        candidates.append(lane)
                for lane in obj.out_lanes:
                    if self.debug:
                        print("OUT: {}\t HEADING {}\tLANE DIRECTION {} ({})".format(lane.label,\
                               self.heading,(lane.direction+180)%360,lane.direction))
                    if math.fabs(lane.direction-self.heading)<45 or  math.fabs(lane.direction-self.heading)>315:
                        candidates.append(lane)
        return candidates


    def testCandidates(self,candidates):
        """Given a list of candidates it tests to see if self in on any of them.
           If there is evidence that self is on a candidate it is put on self.on"""
        for entry in candidates:
            if self.debug:
                print("Testing Candidate {}".format(entry.label))
            if checkOn(self,entry,False): #False for is_on since, by definition, candidates are not in self.on
                self.putOn(entry)


    def checkNewOff(self):
        """Checks if there is evidence that self has left any of the objects
           in self.on"""
        left_right = None
        top_bottom = None
        for obj in list(self.on):
            if self.debug:
                print("Checking if off {}".format(obj.label))
            if not checkOn(self,obj,True): #is_on is True since all objs are by definition in self.on
                self.takeOff(obj)


    def evaluateCrash(self,obj):
        """Checks if the car has crashed into a specified object"""
        obj_coords = obj.four_corners
        #If two cars overlap just right it can be such that the centres overlap, but the corners do not satisfy a crash
        check_points = dict(self.four_corners)
        check_points["centre"] = (self.x_com,self.y_com)
        #print("Vehicle Classes: Evaluating Crash")
        for entry in check_points:
            pt = check_points[entry]
        #    print("Vehicle Classes: Evaluating Crash-{}".format(entry))
        #    print("{} at {}, Four Corners of vehicle at: {}".format(entry,pt,check_points))
        #    print("Front Overlap: {}\tBack Overlap: {}\nLeft Overlap: {}\tRight Overlap: {}".format(\
        #            sideOfLine(pt,obj_coords["front_left"],obj_coords["front_right"]),\
        #            sideOfLine(pt,obj_coords["back_left"],obj_coords["back_right"]),\
        #            sideOfLine(pt,obj_coords["front_left"],obj_coords["back_left"]),\
        #            sideOfLine(pt,obj_coords["front_right"],obj_coords["back_right"])))
        #    print("Vehicle Classes: Done with Evaluation")
            if sideOfLine(pt,obj_coords["front_left"],obj_coords["front_right"])!=\
                sideOfLine(pt,obj_coords["back_left"],obj_coords["back_right"]) and\
                sideOfLine(pt,obj_coords["front_left"],obj_coords["back_left"])!=\
                sideOfLine(pt,obj_coords["front_right"],obj_coords["back_right"]):

                    if self.debug:
                        print("\nVehicle_Classes")
                        print("Crash at {} (obstacle at {})".format((self.x_com,self.y_com),(obj.x_com,obj.y_com)))
                        print("Crash caused from: {} ({})".format(entry,self.four_corners[entry]))
                        print("Front Overlap: {}\tBack Overlap: {}\nLeft Overlap: {}\tRight Overlap: {}".format(\
                                sideOfLine(pt,obj_coords["front_left"],obj_coords["front_right"]),\
                                sideOfLine(pt,obj_coords["back_left"],obj_coords["back_right"]),\
                                sideOfLine(pt,obj_coords["front_left"],obj_coords["back_left"]),\
                                sideOfLine(pt,obj_coords["front_right"],obj_coords["back_right"])))

                        print("Self: {}\nOther: {}".format(self.four_corners,obj.four_corners))
                    return True

        return False


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
                #Only include in potentials obstacles that are interactive
                if entry not in potentials and entry.is_interactive and entry is not self:
                    potentials.append(entry)

        if self.debug:
            print("{} could potentially crash into: {}".format(self.label,[x.label for x in potentials]))

        for veh in potentials:
            crash = self.evaluateCrash(veh)
            if crash:
                has_crashed = True
                crashed.append(veh)

        if self.debug:
            if has_crashed:
                print("{} has crashed into: {}".format(self.label,[x.label for x in crashed]))
            else:
                print("{} has avoided all crashes".format(self.label))

        return has_crashed,crashed


    def checkPositState(self):
        """Check if the vehicle has crashed into any other vehicle/obstacle/run
           off the road and, if not, check if the vehicle has moved to a new road
           object."""

        #First check if you are moving onto a new road section
        if self.debug:
            print("Beginning check of Posit State for: {}".format(self.label))
            print("Currently on: {}".format([x.label for x in self.on]))
            print("Checking On Anything New")
        on_candidates = self.checkNewOn()
        if self.debug:
            print("Done Checking On")
            print("Candidates for Newly on are: {}".format([x.label for x in on_candidates]))
        #Check if you have left anything you were on before
        self.checkNewOff()
        #Check if we are sufficiently "on" any of the candidates identified earlier
        self.testCandidates(on_candidates)
        if self.debug: print("Done Checking Off")

        #Checking if we have wandered off the map
        self.on_road = False
        for entry in self.on:
            #Originally did not include roads here. This is because originally roads were designed to be
            #containers for lanes. However, due to the cyclical nature of checkOn, there is a bug
            # wherein a car can fall between two lanes. It should not be able to fall through a road though
            #if isinstance(entry,road_classes.Lane) or isinstance(entry,road_classes.Junction):
            if isinstance(entry,road_classes.Road) or isinstance(entry,road_classes.Junction):
                self.on_road = True
                break

        if self.debug: print("New list on: {}".format([x.label for x in self.on]))

        #No point repeatedly checking for a crash that already happened
        if self.is_interactive and not self.crashed:
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
                    print("Terminating Simulation")
                    exit(-1)

        #Progress the Trajectory Goal
        on = None
        if self.traj_posit<len(self.trajectory):
            for entry in self.on:
                on = entry
                if isinstance(on,road_classes.Lane):
                    on = on.road
                if on is self.trajectory[self.traj_posit]:
                    self.traj_posit += 1
                    break

        if self.traj_posit == len(self.trajectory):
            if not self.is_ego: self.is_complete = True
            elif self.waypoint_distance is not None and computeDistance([self.x_com,self.y_com],self.waypoints[-1])>self.waypoint_distance:
                self.is_complete = True

        if self.is_complete and self.in_demo and not self.is_ego:
            next_lane = random.choice(on.out_lanes) #You are on a junction and choose a direction by choosing the lane to aim for
            destination = random.choice(next_lane.to_junction.out_lanes).to_junction #Trajectory ends at a junction
            self.trajectory,self.waypoints = trajectory_builder.initialiseTrajectory(next_lane.road,next_lane.is_top_up,destination)
            self.traj_posit = 0
            self.is_complete = False

        if self.traj_posit < len(self.waypoints):
            self.waypoint_distance = computeDistance([self.x_com,self.y_com],self.waypoints[self.traj_posit])
        else: self.waypoint_distance = computeDistance([self.x_com,self.y_com],self.waypoints[-1])

        if self.debug:
            print("End Checking of Posit State for {}\n".format(self.label))


    def resetAfterCrash(self,move_distance=None):
        cr,_ = self.checkForCrash()
        if not self.crashed:
            self.sense()
            if not self.crashed:
                # if self.debug:
                print("Error, no crash detected")
        else:
            cr,_ = self.checkForCrash()
            while self.crashed: #This is in case for some reason a single jump back was not enough to miss the crash
                recheck,crash_list = self.checkForCrash()
                try:
                    crash_veh = crash_list[0] #There should never be a case when this is empty
                    if crash_veh.x_com-self.x_com ==0: #Divide by Zero error
                        if crash_veh.y_com<self.y_com: angle = 90
                        else: angle = 270
                    else:
                        #Negative symbol for y values flips slope to account for inversion of y-coordinate (going "up" decreases y)
                        angle = math.degrees(math.atan(-(crash_veh.y_com-self.y_com)/(crash_veh.x_com-self.x_com)))%360

                    crash_vel = crash_veh.v
                except IndexError: #IndexError throws if crash_list is empty
                    print("Error: Crash detected by {}, but crash_list is empty".format(self.label))
                    # Using generic values to either evade the crash by chance, or else result in a bigger crash which hopefully
                    # avoids the error
                    crash_veh = None
                    angle = self.heading
                    move_distance = self.length/2
                    crash_vel = self.v

                #Compute the angle between ego heading and the location of the agent crashed in to
                rel_angle = max(angle,self.heading)-min(angle,self.heading)
                indicator = math.cos(math.radians(rel_angle)) #If positive other is in front of, otherwise is behind

                if move_distance is None:
                    move_distance = 2.5*self.length

                #Indicator determines whether you should move forwards or backwards to get away from vehicle crashed in to
                self.x_com -= indicator*move_distance*math.cos(math.radians(self.heading))
                self.y_com += indicator*move_distance*math.sin(math.radians(self.heading))
                self.setFourCorners() #Move the rest of the car around the centre of mass

                #Match velocity with other vehicle to avoid immediately crashing again
                self.v = crash_vel
                print("After crash velocity reset to: {}".format(self.v))

                #Assert crash has been avoided an check if this is the case
                self.crashed = False
                if crash_veh is not None:
                    crash_veh.crashed = False

                #Update the states of both agents so they become aware they are no longer crashing
                self.sense()
                if crash_veh is not None:
                    crash_veh.sense()


    def updatePublicState(self):
        state = {}
        state["position"] = (self.x_com,self.y_com)
        state["velocity"] = self.v
        state["heading"] = self.heading
        state["acceleration"] = self.accel
        state["yaw_rate"] = self.yaw_rate
        state["four_corners"] = self.four_corners

        self.pub_state = state


    def updatePrivateState(self):
        state = {}
        self.priv_state = state


    def sense(self):
        """Change the sense variables to match the vehicle's new position/capture
           changes in state."""
        #Perform checks for some of the private state values
        self.checkPositState()
        #Update the Public State values
        self.updatePublicState()
        #Update the Private state values
        self.updatePrivateState()


    def endStep(self):
        """Called at the end of each timestep/iteration of the simulator. Allows for compiling
           of information about what happened during the iteration"""
        #Recording vehicle information to controller log
        if self.controller is not None:
            self.controller.endStep()

        #Trigger key is binary function dependent on the state.
        #If the triggr is true, the consequent is executed
        for trigger in self.triggers.keys():
            if trigger(): self.triggers[trigger]()


    def printStatus(self,mod=""):
        corner_labels = {"back_right":"br","back_left":"bl","front_right":"fr",\
                         "front_left":"fl"}
        dims = ""
        for x in self.four_corners:
            dims += "{}({},{}), ".format(corner_labels[x],round(self.four_corners[x][0],2),\
                                         round(self.four_corners[x][1],2))

        print("{}{}\tON: {}\tHEAD: {}\tSPEED: {}\tACCEL: ({},{})\n{}\t {}".format(mod,\
               self.label,[x.label for x in self.on],self.heading,self.v,self.accel,self.yaw_rate,\
               self.label,dims))


def checkOn(car, obj,is_on=False):
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

    #If you are already on obj, then you should have at least 3 wheels on it to still be "on"
    #But if you are not already confirmed on it then having 2 wheels on it should suffice
    #This results in some gross cyclic behaviour during lane changes, but gets better performance
    #in general
    if is_on: low_bound = 3
    else: low_bound = 2

    #if max(top_bottom,left_right)==4 and min(top_bottom,left_right)>=low_bound:
    if min(top_bottom,left_right)>=low_bound:
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
        if pt[0]>line_strt[0]: return 1 #Right of line
        elif pt[0]<line_strt[0]: return -1 #Left of line
        else: return 0 #On line
    else:
        m = (line_strt[1]-line_end[1])/(line_strt[0]-line_end[0]) #Slope of line
        y_test = line_strt[1] + m*(pt[0]-line_strt[0]) #Height point at pt[0] should be if on line
        if pt[1]>y_test: return 1 #"Below"/Right of line
        elif pt[1]<y_test: return -1 #"Above"/Left of line
        else: return 0 #On line


def angularToCartesianDisplacement(x_disp,y_disp,direction):
    """Translates a displacement in a specified direction into displacement along the
       standard basis axes"""
    phi = math.sqrt(x_disp**2 + y_disp**2)
    delta = math.degrees(math.atan(x_disp/y_disp))
    omega = direction + delta

    return (-phi*math.cos(math.radians(omega)),phi*math.sin(math.radians(\
            omega)))
