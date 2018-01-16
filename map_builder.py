import math
import road_classes

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


def print_contents(junction,mod,last_lane=-1):
    """Program written to test the symbolic architecture of the map. Given a starting
       junction this traverses the map going form junction to connected road to
       junction. The program performs a depth-first search and will ultimately traverse
       every possible road from each junction, in order of occurrence. 
       There is no loop protection here currently. So if loops exist in the map the 
       program will recurse endlessly."""
    print("{}J{}\tIN: {}\tOUT: {}\tWIDTH: {}\tHEIGHT: {}\t({},{})".format(mod,junction.\
           label_TEMP,[entry.label_TEMP for entry in junction.in_lanes],\
           [entry.label_TEMP for entry in junction.out_lanes],\
           round(junction.width,2),round(junction.height,2),\
           round(junction.x,2),round(junction.y,2)))

    for entry in [x for x in junction.out_lanes if x.label_TEMP!= last_lane]:
        print("{}R{}\tLEN: {}\tDIREC: {}\tFROM: {}\t TO: {}\t({},{})\n".format(mod,\
              entry.label_TEMP,round(entry.length,2),entry.direction,\
              entry.from_junction.label_TEMP, entry.to_junction.label_TEMP,\
              round(entry.x,2),round(entry.y,2)))
        print_contents(entry.to_junction,mod+"\t",entry.label_TEMP)


def dimension_solver(dim,angle,cur_dim):
    if angle%180!=0 and 2*dim/math.fabs(math.sin(math.radians(angle)))>cur_dim:
        cur_dim = 2*dim/math.fabs(math.sin(math.radians(angle)))
    return cur_dim


def set_dimensions(junction):
    width = junction.width
    height = junction.height

    for lane in junction.in_lanes:
        if (lane.direction<=45 or lane.direction>=315) or (lane.direction>=135 and lane.direction<=225):
            height = dimension_solver(lane.width,90-lane.direction,height)
        
        else:
            width = dimension_solver(lane.width,lane.direction,width)

    junction.width = width
    junction.height = height
    

def set_anchor_posit(junction,anchor_posit):
   junction.update_coords(anchor_posit[0],anchor_posit[1]) 


def construct_physical_overlay(junctions):
    min_X,min_Y = None,None
    for junction in junctions:
        set_dimensions(junction)

    #NOTE: Before full implementation this should be replaced by "junctions[0]" since
    #      we can't know that there will be at least 4 junctions.
    anchor_junc = junctions[3]
    #Where on the physical screen you want the top-left corner of the chosen junction to 
    #appear
    axis_locat_on_screen = [0,0]
    set_anchor_posit(anchor_junc,axis_locat_on_screen)

    for junction in junctions:
        if min_X is None or junction.x<min_X:
            min_X = junction.x
        if min_Y is None or junction.y<min_Y:
            min_Y = junction.y

    axis_locat_on_screen[0] -= min_X
    axis_locat_on_screen[1] -= min_Y
    set_anchor_posit(anchor_junc,axis_locat_on_screen)
    

junctions = [road_classes.Junction(i) for i in range(4)]

road_angles = [180,0,50]
road_lengths = [30,30,30]
roads = [road_classes.Road(road_lengths[i],road_angles[i],i) for i in range(3)]

set_junc_road_junc(junctions[0],roads[0],junctions[3])
set_junc_road_junc(junctions[3],roads[1],junctions[1])
set_junc_road_junc(junctions[2],roads[2],junctions[3])

for junction in junctions:
    set_dimensions(junction)

construct_physical_overlay(junctions)

print_contents(junctions[1],"")
