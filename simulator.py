import graphic_simulator
import map_builder
import random
import time
import vehicle_classes

#random.seed(49307)

def listFixer(target_list,good_dim,bound_low,bound_high):
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
    if len(target_list) < good_dim[0]:
        print("Insufficient Number of Coordinates Provided")
        while len(target_list) < good_dim[0]:
            if good_dim[1] == 1:
                target_list.append(random.randint(bound_low,bound_high))
            else:
                target_list.append([random.randint(bound_low[i],bound_high[i])\
                                    for i in range(good_dim[1])])

    elif len(target_list)>good_dim[0]:
        print("Number of Coordinates Exceeded Requirements")
        target_list = target_list[:good_dim[0]]

    return target_list


def putCarsOnMap(roads,num_cars,car_speeds=None,car_lanes=None,car_goals=None,debug=True):
    """Constructs the car objects and assigns them coordinates that puts them on either
       specified lanes or else randomly chooses lanes to put them on."""
    cars = []
    lane = None

    #car_lanes is the parameter that specifies what lanes the cars should be put in
    # If car_lanes is None then no lanes have been specified so they will be assigned
    # at random.
    car_lanes = listFixer(car_lanes,[num_cars,2],[0,0],[len(roads)-1,1])
    car_speeds = listFixer(car_speeds,[num_cars,1],1,6)

    #Each entry in car_lanes is the address of a road, and a lane on that road where
    # the car should be placed. Once initialised with this information the Car object
    # will give itself a random location on the specified lane with the same heading as
    # the lane.
    is_ego = True #We want the first car placed to be ego, and all other cars to not be ego
    for i,entry in enumerate(car_lanes):
        cars.append(vehicle_classes.Car(roads[car_lanes[i][0]],car_lanes[i][1],\
                    car_speeds[i],is_ego,i,debug))
        is_ego = False
    return cars


def runSimulation(num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                  car_goals,run_graphics,debug,car_speeds=None,car_lanes=None):
    """This function runs the simulation given the specified parameters. 
        num_junctions: - desired number of junctions in the map being created
        num_roads: - desired number of roads in the map being created
        num_cars: - desired number of cars in the map being created
        road_angles: - angles corresponding to the roads to be created
        road_lengths: - lengths corresponding to the roads to be created
        junc_pairs: - specifies which roads should be linked together. Is a list of 2 entry numeric tuples
                      indicating the meeting of two roads at a junction
        car_goals: - list of integers indicating the junction that each car is to drive towards
        run_graphics: - boolean indicating whether or not the simulator should produce graphical output
        debug: - boolean indicating whether or not the simulator is being debugged. Affects stdout but not run
        car_speeds: - optional parameter that, if provided, specifies the speeds each of the cars travels with
        car_lanes: - optional parameter that, if provided, specifies the lane each car starts in"""
    clock = 0
    runtime = 10.0

    junctions,roads = map_builder.buildMap(num_junctions,num_roads,road_angles,\
                                           road_lengths,junc_pairs)

    cars = putCarsOnMap(roads,num_cars,car_speeds,car_lanes,car_goals,debug)

    if run_graphics:
        g_sim = graphic_simulator.GraphicSimulator(junctions,roads,cars)

    if debug:
        map_builder.printContents(junctions[0])
        print("\n")
    t0 = time.time()
    while(clock<runtime):
        if debug: print("TIME: {}".format(clock))
        for entry in cars:
            entry.move(0,0)
            entry.sense()
            if debug:
                print("\nEnd Of Round Status Update:")
                entry.printStatus()
                print("\n")
        if run_graphics:
            g_sim.update()
    
        clock += .1
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
    num_junctions = 4
    num_roads = 3
    num_cars = 5
    
    road_angles = [180,0,50]
    road_lengths = [30,30,30]

    junc_pairs = [(0,3),(3,1),(2,3)]

    car_speeds = [5.5,0]
    car_lanes = [(0,1),(1,1)]
    car_goals = [0,3,3,3,3]

    run_graphics = True
    debug = False
    runSimulation(num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                  car_goals,run_graphics,debug)
