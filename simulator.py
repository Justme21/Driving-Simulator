import controller_classes
import datetime
import graphic_simulator
import map_builder
import matplotlib.pyplot as plt
import random
import time
import vehicle_classes

#random.seed(49307)
MAX_RUNTIME = 1000

class Simulator():
    def __init__(self,graphic=False,runtime=None,debug=False,dt=.1):
        self.junctions = []
        self.cars = []
        self.roads = []

        self.obj_dict = {'road':self.roads,'car':self.cars,'junction':self.junctions}

        self.time = None
        self.runtime = runtime

        self.graphic = graphic
        self.g_sim = None

        self.debug = debug
        self.dt = dt


    def loadCars(self,car_list):
        if not isinstance(car_list,list): car_list = [car_list]
        self.obj_dict['car'] += car_list


    def runSensing(self):
        for car in self.cars:
            car.sense()


    def initialiseSimulator(self,num_junctions,num_roads,road_angles,road_lengths,junc_pairs,init_speeds,init_lanes,starts,dests):
        if self.cars != []:
            self.junctions,self.roads,self.cars = constructEnvironment(num_junctions,num_roads,road_angles,road_lengths,\
                                                    junc_pairs,self.cars,init_speeds,init_lanes,starts,dests)

            if self.graphic: self.g_sim = graphic_simulator.GraphicSimulator(self.junctions,self.roads,self.cars)
            self.runSensing()
            print("Initialisation Complete")
        else:
            print("Error: No Cars Loaded")
            exit(-1)

        if self.debug:
            map_builder.printContents(self.junctions[0])
            print("HERE NOW")
            for entry in self.junctions[0].out_lanes:
                print(entry.label)
            print("\n")


    def singleStep(self):
        for car in self.cars:
            car.chooseAction()
            car.move()
        if self.graphic: self.g_sim.update()
        self.runSensing()


    def runComplete(self):
        self.time = 0
        while canGo(self.cars):
            if (self.runtime is not None and self.time>self.runtime) or self.time>MAX_RUNTIME:
                break
            self.singleStep()
            self.time += self.dt
            self.endStep()


    def endStep(self):
        if self.debug:
            for car in self.cars:
                print("\nEnd Of Round Status Update:")
                car.printStatus()
                print("\n")


    def wrapUp(self):
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

    for i,entry in enumerate(target_list):
        if len(entry)<good_dim[1]:
            if good_dim[1] == 2:
                target_list[i] = entry+[random.choice(choice_list[j])\
                                     for j in range(len(entry),good_dim[1])]

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
    lane = None

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
        car.controller.restartModel()
    return cars


def constructEnvironment(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,\
                         cars,car_speeds,car_lanes,start,destinations):

    junctions,road_dict = map_builder.buildMap(num_junctions,num_roads,road_angles,\
                                           road_lengths,junc_pairs)

    cars = putCarsOnMap(cars,road_dict,start,destinations,car_speeds)
    roads = list(road_dict.values())
    return junctions,roads,cars


def canGo(cars):
    if cars[0].is_complete: return False
    for car in cars:
        if car.crashed or not car.on_road: return False
    return True


def initialiseControlledCars(num_cars,controllers,accel_cats,angle_cats,debug):
    cars = []
    controllers += [controller_classes.RandomController(accel_cats,angle_cats) for _ in range(num_cars-len(controllers))]
    is_ego = True
    for i in range(num_cars):
        cars.append(vehicle_classes.Car(controllers[i],is_ego,i,debug))
        is_ego = False
    return cars


def storePerformance(performance_list,step_size):
    now = datetime.datetime.now()
    perf_record = open("performance-{}-{}-{}.txt".format(now.day,now.month,now.year),"w")
    for i in range(len(performance_list)):
        perf_record.write("{}\t{}\n".format(i*step_size,performance_list[i]))
    perf_record.close()

    plt.figure(2)
    plt.clf()
    plt.title('Distance to Goal')
    plt.xlabel('Episode')
    plt.ylabel('Distance')
    plt.plot([x*10 for x in range(len(performance_list))],performance_list)
    plt.show()


def runTraining(num_episodes,num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                controllers,accel_cats,angle_cats,run_graphics,debug,car_speeds=None,car_lanes=None):

    dist_to_obj = 0
    performance_list = []

    cur_states = [None for _ in range(num_cars)]
    next_states = [None for _ in range(num_cars)]

    cars = initialiseControlledCars(num_cars,controllers,accel_cats,angle_cats,debug)
    simulator = Simulator(run_graphics,dt=.1)
    simulator.loadCars(cars)

    for ep_num in range(num_episodes):
        #junctions,roads,cars = constructEnvironment(num_junctions,num_roads,road_angles,road_lengths,\
        #                                            junc_pairs,cars,car_speeds,car_lanes,debug)
        simulator.initialiseSimulator(num_junctions,num_roads,road_angles,road_lengths,\
                                                    junc_pairs,car_speeds,car_lanes,starts,dests)

        for i,car in enumerate(cars):
            cur_states[i] = car.composeState()

        while canGo(cars):
            simulator.singleStep()
            simulator.runSensing()
            for i,car in enumerate(cars):
                next_states[i] = car.composeState()
                car.controller.train(cur_states[i],next_states[i])
            cur_states = list(next_states)

        for car in cars:
            if not isinstance(car.controller,controller_classes.RandomController):
                dist_to_obj += car.distToObj()

        print("EP:{}\tDIST:{}\t{}".format(ep_num,cars[0].distToObj(),cars[0].controller.reward_sum))
        if ep_num%10==0:
            performance_list.append(dist_to_obj/10)
            dist_to_obj = 0


        simulator.wrapUp()


    for car in cars:
        car.controller.recordModel()
        car.controller.storeResults()

    storePerformance(performance_list,10)


def runSimulation(num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                  accel_cats,angle_cats,run_graphics,debug,starts,dests,car_speeds=None,car_lanes=None):
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
    runtime = 10.0

    cars = initialiseControlledCars(num_cars,[],accel_cats,angle_cats,debug)
    simulator = Simulator(run_graphics,runtime,debug,dt=.1)
    simulator.loadCars(cars)

    simulator.initialiseSimulator(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,\
                                                car_speeds,car_lanes,starts,dests)

    t0 = time.time()
    simulator.runComplete()
    t1 = time.time()
    print("Runtime is {}".format(t1-t0))


if __name__ == "__main__":
    #Figure of eight
    #num_junctions = 6
    #num_roads = 7
    #num_cars = 1

    #road_angles = [90,90,180,180,180,90,90]
    #road_lengths = [30,30,30,30,30,30,30]

    #junc_pairs = [(0,1),(1,2),(3,2),(4,1),(5,0),(4,3),(5,4)]

    #3-road intersection
    num_junctions = 6
    num_roads = 5
    num_cars = 1

    road_angles = [180,180,180,180,180]
    road_lengths = [30,30,30,30,30]

    junc_pairs = [(0,1),(1,2),(2,3),(3,4),(4,5)]
    starts = [[(0,1)]]
    dests = [[(4,5)]]

    car_speeds = [5.5]
    car_lanes = [(0,1)]

    run_graphics = False
    debug = True

    num_episodes = 10000
    accel_cats = [(-3,-1.5),(-1.5,0),(0,1.5),(1.5,3)]
    angle_cats = [(-5,-2.5),(-2.5,0),(0,2.5),(2.5,5)]
    #controllers = []
    #controllers = [controller_classes.DQNController("safe",accel_cats,angle_cats)]
    runSimulation(num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                  accel_cats,angle_cats,run_graphics,debug,starts,dests)
    #runTraining(num_episodes,num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
    #              controllers,accel_cats,angle_cats,run_graphics,debug,starts,dests,car_speeds,car_lanes)
