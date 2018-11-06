import controller_classes
import datetime
import graphic_simulator
import map_builder
import matplotlib.pyplot as plt
import random
import time
import vehicle_classes

import math

#random.seed(49307)
MAX_RUNTIME = 1000

class Simulator():
    def __init__(self,run_graphics=False,draw_traj=False,runtime=None,debug=False,dt=.1):
        """run_graphics: [bool] indicates if graphical simulation is being run
           draw_traj: [bool] indicates that trajectories for vehicles should be drawn
           runtime: [float/None] the expected runtime of the simulation. If None then runs until terminated
           debug: [bool] if True then prints statements littered throughout code for debug purposes
           dt: [float] the length (in seconds) of a single timestep in the simulation"""
        self.junctions = [] #list containing all the junctions in the simulation
        self.cars = [] #List of all the cars in the simulation
        self.roads = [] #List of all the roads (linking junctions) in the simulation

        #All the above lists are stored in a dictionary for convenience
        self.obj_dict = {'road':self.roads,'car':self.cars,'junction':self.junctions}

        self.time = None #Keeps track of simulation time
        self.runtime = runtime #How long the simulation should run for if not terminated prematurely

        self.graphic = run_graphics #Boolean, whether or not graphics are generated during simulation
        self.traj = run_graphics and draw_traj #Won't try draw trajectory if not graphic
        self.g_sim = None #If graphics are being used g_sim will be initalised as the graphical component

        self.debug = debug #Debug prints debug text littered throughout the code
        self.dt = dt #size of a timestep in the simulator

        self.min_dist = []


    def loadCars(self,car_list):
        """Loads a list of initialised cars into the simulator.
           car_list: [vehicle_classes.Car/list] the cars to be loaded into the simulator"""
        if not isinstance(car_list,list): car_list = [car_list] #if only a single car is passed it is wrapped in list
        self.obj_dict['car'] += car_list


    def runSensing(self):
        """Runs sense command for each car in the simulator. Happens in a single timestep so is simultaneous for all vehicles"""
        for car in self.cars:
            car.sense()


    def drawSimulation(self):
        """If graphics are being used then initialises the graphical simulator and draws the current scene"""
        if self.g_sim is None:
            self.g_sim = graphic_simulator.GraphicSimulator(self.junctions,self.roads,self.cars,self.traj)
        self.g_sim.drawScene()


    def initialiseSimulator(self,num_junctions,num_roads,road_angles,road_lengths,junc_pairs,init_speeds,starts,dests):
        """Given the relevant objects initialises the simulator and loads cars in.
           num_junctions: [int] the number of junctions used in the simulation
           num_roads: [int] the number of roads used in the simulation
           road_angles: [list] list containing angles associated with each road in the simulation
           road_lengths: [list] list containing lengths associated with each road in the simulation
           junc_pairs: [list] list of tuples where each tuple indicates which junctions are linked to each other by roads
           init_speeds: [list] the initial speeds of each car in the simulation
           starts: [list] list of tuples where each tuple is a junc_pair identifying the road the vehicle should start on.
                   the list also contains a binary integer indicating whether the car should be on the left lane of the road.
           dests: [list] list of tuples where each tuple is a junc_pair identifying the junction the vehicle should aim to end on.
                   the list also contains a binary integer indicating whether the car is targeting the second junction in the tuple."""
        if self.cars != []:
            self.junctions,self.roads,self.cars = constructEnvironment(num_junctions,num_roads,road_angles,road_lengths,\
                                                    junc_pairs,self.cars,init_speeds,starts,dests)

            if self.graphic:
                self.g_sim = graphic_simulator.GraphicSimulator(self.junctions,self.roads,self.cars,self.traj)
            self.runSensing() #Sensing happens at the end of each timestep, so initial round of sensing must occur
            if self.debug:
                print("Initialisation Complete")
        else:
            print("Error: No Cars Loaded")
            exit(-1)

        if self.debug:
            map_builder.printContents(self.junctions[0])


    def singleStep(self,move_dict=None,index=None):
        """Runs a single timestep of the simulator. Each car chooses an action and moves according to the action.
           After all cars have moved the graphical element is updated and all cars sense the changes in the environment
           move_dict: dictionary with Car objects as keys that maps to a sequential list of (lin_accel,ang_accel) pairs
           index: integer indicating the entry from the list of actions that should be taken for each car"""
        for car in self.cars:
            if move_dict is None or car not in move_dict:
                car.chooseAction()
            else:
                if index<len(move_dict[car]):
                    car.setAction(move_dict[car][index][0],move_dict[car][index][1])
                else:
                    car.setAction(0,0)
            car.move()
        dist = math.sqrt((self.cars[0].x_com-self.cars[1].x_com)**2 + (self.cars[0].y_com-self.cars[1].y_com)**2)
        self.min_dist.append(dist)
        if self.graphic: self.g_sim.update()
        self.runSensing()


    def runComplete(self,move_dict=None):
        """Runs a complete runthrough of the simulation. If a runtime has been specified will run until time exceeds runtime. Otherwise
           will run until exceeds the global, MAX_RUNTIME.
           move_dict: dictionary with Car objects as keys that map to a sequential list of (lin_accel,ang_accel) pairs"""
        self.time = 0
        if move_dict is not None:
            i=0
        while canGo(self.cars):
            if (self.runtime is None and self.time>MAX_RUNTIME) or (self.runtime is not None and self.time>self.runtime):
            #if (self.runtime is not None and self.time>self.runtime) or self.time>MAX_RUNTIME:
                break
            if move_dict is None:
                self.singleStep()
            else:
                try:
                    self.singleStep(move_dict,i)
                    i+=1
                except IndexError: #messy way to break out of while loop when car can still move but out of entries in trajectory
                    break
            self.time += self.dt
            self.endStep()


    def endStep(self):
        """Prints debug output at end of each timestep, mainly for debugging purposes"""
        if self.debug:
            for car in self.cars:
                print("\nEnd Of Round Status Update:")
                car.printStatus()
                print("\n")


    def wrapUp(self):
        """Shuts down the graphical part of the simulator if it has been initialised"""
        if self.graphic:
            time.sleep(2)
            self.g_sim.shutdown()


def listFixer(target_list,good_dim,choice_list):
    """Fixes the list provided as parameter so that it satisfies the requirements
       (i.e. has correct dimensionality). If the list is too long elements are removed.
       If too long then new, random entries are created using the upper and lower bound values to
       generate appropriate values.
       taget_list: - the list that is being 'fixed'
       good_dim: - parameter containing the correct dimensions for the list
                   if list is 1D then contains length, otherwise is list that contains the
                   length each dimension should be
        bound_low: - the lower bound for each entry in the list
                   if list is 1D then this is a scalar value, otherwise it is a list of values
        bound_high: - same as bound_low but with upper bound values"""
    if target_list is None:
        target_list = []

    #This is cheating but we assume that lists have at least one element, and the first element
    # is consistent with the type for the full list. Done as int etc. do not have "len" defined
    if len(target_list)>0:
        try: #Handle the case where the list contents are numbers and therefore, presumably, fine
            for i,entry in enumerate(target_list):
                if len(entry)<good_dim[1]:
                    if good_dim[1] == 2:
                        target_list[i] = entry+[random.choice(choice_list[j])\
                                             for j in range(len(entry),good_dim[1])]
        except TypeError: #This will fire if len(entry) is run on an int or a float
            pass

    if len(target_list) < good_dim[0]:
        print("Insufficient Number of Coordinates Provided")
        while len(target_list) < good_dim[0]:
            if good_dim[1] == 1:
                target_list.append(random.choice(choice_list))
            else:
                target_list.append([random.choice(choice_list[i])\
                                    for i in range(good_dim[1])])

    elif len(target_list)>good_dim[0]:
        print("Number of Coordinates Exceeded Requirements")
        target_list = target_list[:good_dim[0]]

    return target_list


def putCarsOnMap(cars,road_dict,start=None,dest_list=None,car_speeds=None):
    """Constructs the car objects and assigns them coordinates that puts them on either
       specified lanes or else randomly chooses lanes to put them on."""
    num_cars = len(cars)

    #car_lanes is the parameter that specifies what lanes the cars should be put in
    # If car_lanes is None then no lanes have been specified so they will be assigned
    # at random.
    start = listFixer(start,[num_cars,2],(list(road_dict.keys()),[0,1]))
    dest_list = listFixer(dest_list,[num_cars,2],(list(road_dict.keys()),[0,1]))
    car_speeds = listFixer(car_speeds,[num_cars,1],[x for x in range(1,6)])

    #Each entry in car_lanes is the address of a road, and a lane on that road where
    # the car should be placed. Once initialised with this information the Car object
    # will give itself a random location on the specified lane with the same heading as
    # the lane.
    for i,car in enumerate(cars):
        car.initSetup((road_dict[start[i][0]],start[i][1]),(road_dict[dest_list[i][0]],dest_list[i][1]),car_speeds[i])
    return cars


def constructEnvironment(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,\
                         cars,car_speeds,start,destinations):
    """Given the attributes for the environment builds an approprite map and populates it with cars.
       num_junctions: [int] the number of junctions in the simulation
       num_roads: [int] the number of roads in the simulation
       road_angles: [list] list of angles associated with each road in the simulation
       road_lengths: [list] list of lengths associated with each road in the simulation
       junc_pairs: [list] list of tuples where each tuple identifies a pair of junctions that are linked by a road
       cars: [list] list of vehicle_classes.Car objects that have already been initialised
       car_speeds: [list] list of speeds for each car in cars
       start: [list] list of tuples where each tuple is a junc_pair identifying the road the vehicle should start on.
              the list also contains a binary integer indicating whether the car should be on the left lane of the road.
       dests: [list] list of tuples where each tuple is a junc_pair identifying the junction the vehicle should aim to end on.
              the list also contains a binary integer indicating whether the car is targeting the second junction in the tuples"""
    junctions,road_dict = map_builder.buildMap(num_junctions,num_roads,road_angles,\
                                           road_lengths,junc_pairs)

    cars = putCarsOnMap(cars,road_dict,start,destinations,car_speeds)
    roads = list(road_dict.values())
    return junctions,roads,cars


def canGo(cars):
    """Indicates whether the simulator should continue or terminate"""
    if cars[0].is_complete:
        return False #If the ego vehicle (default cars[0]) has completed its objective it can go
    for car in cars:
        if car.crashed or not car.on_road:
            return False #If a car has crashed the simulation should stop
    return True


def initialiseControlledCars(num_cars,controllers,accel_range,angle_range,debug):
    """Initialises the specified number of cars with the indicated controllers. Any cars with unspecified controllers
       get random action controllers by default. First car initialised (with the first provided controller) is the ego
       vehicle by default.
       num_cars: [int] number of cars to be initialised
       controllers: [list] list of controller_classes objects to be assigned to the cars
       accel_range: [list] list indicating the lower and upper range for the linear acceleration
       angle_range: [list] list inidicating the lower and upper range for the angular acceleration
       debug: [bool] indicates whether or not debug print statements should be printed"""
    cars = []
    controllers += [controller_classes.RandomController(accel_range,angle_range) for _ in range(num_cars-len(controllers))]
    is_ego = True
    for i in range(num_cars):
        cars.append(vehicle_classes.Car(controllers[i],is_ego,i,debug))
        is_ego = False
    return cars


def runSimulation(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,num_cars,controllers,\
                  accel_range,angle_range,run_graphics,draw_traj,debug,starts,dests,car_speeds=None):
    """This function runs the simulation given the specified parameters.
        num_junctions: - desired number of junctions in the map being created
        num_roads: - desired number of roads in the map being created
        num_cars: - desired number of cars in the map being created
        road_angles: - angles corresponding to the roads to be created
        road_lengths: - lengths corresponding to the roads to be created
        junc_pairs: - specifies which roads should be linked together. Is a list of 2 entry numeric tuples
                      indicating the meeting of two roads at a junction
        accel_cats: - the ranges from which the controller for the cars can select the acceleration
        angle_cats: - the ranges from which the controller for the cars can select the angle change
        run_graphics: - boolean indicating whether or not the simulator should produce graphical output
        debug: - boolean indicating whether or not the simulator is being debugged. Affects stdout but not run
        car_speeds: - optional parameter that, if provided, specifies the speeds each of the cars travels with
        car_lanes: - optional parameter that, if provided, specifies the lane each car starts in"""
    clock = 0
    runtime = 20.0

    cars = initialiseControlledCars(num_cars,controllers,accel_range,angle_range,debug)
    simulator = Simulator(run_graphics,draw_traj,runtime,debug,dt=.1)
    simulator.loadCars(cars)

    simulator.initialiseSimulator(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,\
                                                car_speeds,starts,dests)

    t0 = time.time()
    simulator.runComplete()
    t1 = time.time()
    print("Runtime is {}".format(t1-t0))


if __name__ == "__main__":
    #Figure of eight
    #num_junctions = 6
    #num_roads = 7
    #num_cars = 5

    #road_angles = [90,90,180,180,180,90,90]
    #road_lengths = [50,30,15,15,15,30,50]

    #junc_pairs = [(0,1),(1,2),(3,2),(4,1),(5,0),(4,3),(5,4)]

    #3-road intersection
    num_junctions = 5
    num_roads = 4
    num_cars = 2

    road_angles = [90,90,90,90]
    road_lengths = [10,10,50,15]

    junc_pairs = [(0,1),(1,2),(2,3),(3,4)]
    starts = [[(1,2),1],[(0,1),1]]
    #dests = [[(0,1)]]
    dests = [[(2,3),1],[(3,4),1]]

    car_speeds = [0,0]

    run_graphics = True
    draw_traj = True
    debug = False

    num_episodes = 10000
    accel_range = [-3,3]
    angle_range = [-5,5]
    controllers = []
    runSimulation(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,num_cars,controllers,\
                  accel_range,angle_range,run_graphics,draw_traj,debug,starts,dests,car_speeds)
