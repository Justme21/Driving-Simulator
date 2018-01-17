import math
import random
import road_classes
import vehicle_classes

random.seed(49307)

def set_road_to_junc(road,junc,entering):
    """Performs the appropriate pairing of a road object with a junction it leads into.
       'entering' is a boolean variable that is true if when travelling along the road
       in the left lane going from left to right you enter the junction."""
    if entering:
        road.top_up_lane.to_junction = junc
        road.bottom_down_lane.from_junction = junc
        junc.in_lanes.append(road.top_up_lane)
        junc.out_lanes.append(road.bottom_down_lane)
    else:
        road.top_up_lane.from_junction = junc
        road.bottom_down_lane.to_junction = junc
        junc.out_lanes.append(road.top_up_lane)
        junc.in_lanes.append(road.bottom_down_lane)


def set_junc_road_junc(from_junc,road,to_junc):
    """Given two junctions and the road between them, pairs the road with each
       junction as appropriate.
       For inputs order is 'from_junc-road-to_junc' where 'from_junc' is the junction
       being exited if in the left lane travelling from left to right on the map."""
    set_road_to_junc(road,from_junc,0)
    set_road_to_junc(road,to_junc,1)


def print_contents(junction,mod="",prev_lanes=[]):
    """Program written to test the symbolic architecture of the map. Given a starting
       junction this traverses the map going form junction to connected road to
       junction. The program performs a depth-first search and will ultimately traverse
       every possible road from each junction, in order of occurrence. """

    junction.print_status(mod)

    for entry in junction.out_lanes:
        if entry.label not in prev_lanes:
            entry.print_status(mod)
            prev_lanes.append(entry.label)
            print_contents(entry.to_junction,mod+"\t",prev_lanes)


def dimension_solver(dim,angle,cur_dim):
    """Identifies the largest value for the dimension of a junction (height or width)
       by storing the max and examining the angles of all the roads entering into the
       junction"""
    if angle%180!=0 and 2*dim/math.fabs(math.sin(math.radians(angle)))>cur_dim:
        cur_dim = 2*dim/math.fabs(math.sin(math.radians(angle)))
    return cur_dim


def set_dimensions(junction):
    """Determines the appropriate height and width for the junction by examining the
       roads that enter into it. This happens by splitting the entering lanes into 
       top/bottom and left/right entering and then determining the angle at which 
       each meets the edge of the junction"""
    width = junction.width
    height = junction.height

    for lane in junction.in_lanes:
        if (lane.direction<=45 or lane.direction>=315) or \
              (lane.direction>=135 and lane.direction<=225):
            height = dimension_solver(lane.width,90-lane.direction,height)
        
        else:
            width = dimension_solver(lane.width,lane.direction,width)

    junction.width = width
    junction.height = height
    

def set_anchor_posit(junction,anchor_posit):
   """Given an arbitrary initial node (junction) and position for that node on 
      the (x,y) plane, assigns the location for every road and junction on the graph."""
   junction.update_coords(anchor_posit[0],anchor_posit[1]) 


def construct_physical_overlay(junctions):
    """Takes the symbolic road-junction network (represented by the list of all 
       junctions in the network since the roads uniquely connect junctions) and
       constructs an appropriate equivalent on the (x,y) plane. """
    min_X,min_Y = None,None
    #Specify the width and height of each junction
    for junction in junctions:
        set_dimensions(junction)

    anchor_junc = junctions[0]
    #Arbitrary anchor point
    anchor_point = [0,0]
    set_anchor_posit(anchor_junc,anchor_point)

    for junction in junctions:
        if min_X is None or junction.x<min_X:
            min_X = junction.x
        if min_Y is None or junction.y<min_Y:
            min_Y = junction.y

    #Tweaking of the values being subtracted here would allow you to put the 
    # graph where you like however, this might require knowing the dimension
    # of the entire space.... or else just performing 2 updates, one to update
    # the x values and one to update the y's
    anchor_point[0] -= min_X
    anchor_point[1] -= min_Y
    set_anchor_posit(anchor_junc,anchor_point)
    

def build_map(num_junctions,num_roads,num_cars,road_angles,road_lengths,junc_pairs,\
                 car_lanes=None):
    #junctions = [road_classes.Junction(i) for i in range(6)]

    #road_angles = [90,90,180,180,180,90,90]
    #road_lengths = [1,1,1,1,1,1,1]
    #roads = [road_classes.Road(road_lengths[i],road_angles[i],i) for i in range(7)]

    #set_junc_road_junc(junctions[0],roads[0],junctions[1])
    #set_junc_road_junc(junctions[1],roads[1],junctions[2])
    #set_junc_road_junc(junctions[3],roads[2],junctions[2])
    #set_junc_road_junc(junctions[4],roads[3],junctions[1])
    #set_junc_road_junc(junctions[5],roads[4],junctions[0])
    #set_junc_road_junc(junctions[4],roads[5],junctions[3])
    #set_junc_road_junc(junctions[5],roads[6],junctions[4])


    junctions = [road_classes.Junction(i) for i in range(num_junctions)]

    roads = [road_classes.Road(road_lengths[i],road_angles[i],i) for i in range(num_roads)]

    for i in range(num_roads):
        set_junc_road_junc(junctions[junc_pairs[i][0]],roads[i],\
                              junctions[junc_pairs[i][1]])

    construct_physical_overlay(junctions)

    cars = []
    lane = None

    if car_lanes is None:
        car_lanes = []
        for _ in range(num_cars):
            car_lanes.append((random.randint(0,num_roads-1),random.randint(0,1)))

    for i,entry in enumerate(car_lanes):
        if entry[1]:
            lane = roads[entry[0]].top_up_lane        
        else:
            lane = roads[entry[0]].bottom_down_lane
        cars.append(vehicle_classes.Car(lane,i))
   
    return cars,junctions,roads 
