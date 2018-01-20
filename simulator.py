import map_builder
import random
import time
import vehicle_classes

random.seed(493072)


def putCarsOnMap(roads,num_cars,car_lanes=None):
    """Constructs the car objects and assigns them coordinates that puts them on either
       specified lanes or else randomly chooses lanes to put them on."""
    cars = []
    lane = None

    #car_lanes is the parameter that specifies what lanes the cars should be put in
    # If car_lanes is None then no lanes have been specified so they will be assigned
    # at random.
    if car_lanes is None:
        car_lanes = []
        for _ in range(num_cars):
            car_lanes.append((random.randint(0,num_roads-1),random.randint(0,1)))

    #Each entry in car_lanes is the address of a road, and a lane on that road where
    # the car should be placed. Once initialised with this information the Car object
    # will give itself a random location on the specified lane with the same heading as
    # the lane.
    for i,entry in enumerate(car_lanes):
        cars.append(vehicle_classes.Car(roads[car_lanes[i][0]],car_lanes[i][1],i))
    return cars

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
num_cars = 1

road_angles = [180,0,50]
road_lengths = [30,30,30]

junc_pairs = [(0,3),(3,1),(2,3)]

junctions,roads = map_builder.buildMap(num_junctions,num_roads,road_angles,road_lengths,\
                                        junc_pairs)

cars = putCarsOnMap(roads,num_cars)


map_builder.printContents(junctions[0])
print("\n")
while(True):
    for entry in cars:
        entry.move(0,0)
        entry.printStatus()
        print("\n")
    time.sleep(.1)
