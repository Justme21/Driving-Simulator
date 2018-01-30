import graphic_classes
import map_builder
import pygame
import random
import time
import vehicle_classes

random.seed(493072)
screen_width,screen_height = 1024,640

WHITE = (255,255,255)

def putCarsOnMap(roads,num_cars,car_speeds=None,car_lanes=None):
    """Constructs the car objects and assigns them coordinates that puts them on either
       specified lanes or else randomly chooses lanes to put them on."""
    cars = []
    lane = None

    #car_lanes is the parameter that specifies what lanes the cars should be put in
    # If car_lanes is None then no lanes have been specified so they will be assigned
    # at random.
    if car_lanes is None:
        car_lanes = []
        car_speeds = []
        for _ in range(num_cars):
            car_lanes.append((random.randint(0,num_roads-1),random.randint(0,1)))
            car_speeds.append(5.5)

    #Each entry in car_lanes is the address of a road, and a lane on that road where
    # the car should be placed. Once initialised with this information the Car object
    # will give itself a random location on the specified lane with the same heading as
    # the lane.
    for i,entry in enumerate(car_lanes):
        cars.append(vehicle_classes.Car(roads[car_lanes[i][0]],car_lanes[i][1],\
                    car_speeds[i],i))
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


clock = 0
runtime = 15

junctions,roads = map_builder.buildMap(num_junctions,num_roads,road_angles,road_lengths,\
                                        junc_pairs)

car_speeds = [5.5,0]
car_lanes = [(0,1),(1,1)]
cars = putCarsOnMap(roads,num_cars,car_speeds,car_lanes)

g_junc = [graphic_classes.GraphicWrapper(x) for x in junctions]
g_road = [graphic_classes.GraphicWrapper(x) for x in roads]
g_cars = [graphic_classes.GraphicWrapper(x) for x in cars]

junc_list = pygame.sprite.Group()
road_list = pygame.sprite.Group()
car_list = pygame.sprite.Group()

for entry in g_junc:
    entry.draw()
    junc_list.add(entry)
for entry in g_road: 
    entry.draw()
    road_list.add(entry)
for entry in g_cars:
    entry.draw()
    car_list.add(entry)

pygame.init()
screen = pygame.display.set_mode([screen_width,screen_height])
background = pygame.Surface(screen.get_size())
background.fill(WHITE)
g_clock = pygame.time.Clock()
done = False

map_builder.printContents(junctions[0])
print("\n")
t0 = time.time()
while not done:
    print("TIME: {}".format(clock))
    for entry in cars:
        entry.move(0,0)
        entry.sense()
        print("\nEnd Of Round Status Update:")
        entry.printStatus()
        print("\n")

    for entry in g_junc:
        entry.draw()
        junc_list.add(entry)
    for entry in g_road: 
        entry.draw()
        road_list.add(entry)
    for entry in g_cars:
        entry.draw()
        car_list.add(entry)
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
    
    screen.blit(background,(0,0))
    pygame.display.update()
    
    road_list.draw(screen)
    junc_list.draw(screen)
    car_list.draw(screen)
    
    g_clock.tick(10)
    pygame.display.flip()
    clock += .1
    #time.sleep(.1)
    if clock >= runtime:
        done = True

pygame.quit()
t1 = time.time()
print("Runtime is {}".format(t1-t0))
