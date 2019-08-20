import math
import random
import road_classes

#NOTE: In joining a road to a junction must add length to a lane so that
#      both lanes meet the junction
def setRoadToJunc(road,junc,entering):
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


def setJuncRoadJunc(from_junc,road,to_junc):
    """Given two junctions and the road between them, pairs the road with each
       junction as appropriate.
       For inputs order is 'from_junc-road-to_junc' where 'from_junc' is the junction
       being exited if in the left lane travelling from left to right on the map."""
    setRoadToJunc(road,from_junc,0)
    setRoadToJunc(road,to_junc,1)


def printContents(junction,mod="",prev_lanes=[]):
    """Program written to test the symbolic architecture of the map. Given a starting
       junction this traverses the map going form junction to connected road to
       junction. The program performs a depth-first search and will ultimately traverse
       every possible road from each junction, in order of occurrence. """

    junction.printStatus(mod)

    for entry in junction.out_lanes:
        if entry.label[:-1] not in prev_lanes:
            entry.printStatus(mod)
            prev_lanes.append(entry.label[:-1])
            printContents(entry.to_junction,mod+"\t",prev_lanes)


def dimensionSolver(dim,angle,cur_dim):
    """Identifies the largest value for the dimension of a junction (height or width)
       by storing the max and examining the angles of all the roads entering into the
       junction"""
    if angle%180!=0 and 2*dim/math.fabs(math.sin(math.radians(angle)))>cur_dim:
        cur_dim = 2*dim/math.fabs(math.sin(math.radians(angle)))
    return cur_dim


def setDimensions(junction):
    """Determines the appropriate height and width for the junction by examining the
       roads that enter into it. This happens by splitting the entering lanes into 
       top/bottom and left/right entering and then determining the angle at which 
       each meets the edge of the junction"""
    width = junction.width
    height = junction.length

    for lane in junction.in_lanes:
        if (lane.direction<=45 or lane.direction>=315) or \
              (lane.direction>=135 and lane.direction<=225):
            height = dimensionSolver(lane.width,90-lane.direction,height)

        else:
            width = dimensionSolver(lane.width,lane.direction,width)

    junction.width = width
    junction.length = height


def setAnchorPosit(junction,anchor_posit):
   """Given an arbitrary initial node (junction) and position for that node on 
      the (x,y) plane, assigns the location for every road and junction on the graph.
      The first parameter passed to "updateCoords" is which corner of the node (by
      the junction's orientation) you are setting"""
   junction.updateCoords(anchor_posit) 


def constructPhysicalOverlay(junctions):
    """Takes the symbolic road-junction network (represented by the list of all 
       junctions in the network since the roads uniquely connect junctions) and
       constructs an appropriate equivalent on the (x,y) plane. """
    min_X,min_Y = None,None
    #Specify the width and height of each junction
    for junction in junctions:
        setDimensions(junction)

    anchor_junc = junctions[0]
    #Arbitrary anchor point
    anchor_point = [0,0]
    setAnchorPosit(anchor_junc,anchor_point)

    for junc in junctions:
        min_corner_x = min([junc.four_corners[x][0] for x in junc.four_corners])
        min_corner_y = min([junc.four_corners[x][1] for x in junc.four_corners]) 
        if min_X is None or min_corner_x<min_X:
            min_X = min_corner_x
        if min_Y is None or min_corner_y<min_Y:
            min_Y = min_corner_y

    #Tweaking of the values being subtracted here would allow you to put the 
    # graph where you like however, this might require knowing the dimension
    # of the entire space.... or else just performing 2 updates, one to update
    # the x values and one to update the y's
    anchor_point[0] -= min_X
    anchor_point[1] -= min_Y
    setAnchorPosit(anchor_junc,anchor_point)


def buildMap(num_junctions,num_roads,road_angles,road_lens,junc_pairs,lane_width=None):
    """Runs the map builder. Constructs the junctions and roads and initiates the
       process of linking them together."""
    junctions = [road_classes.Junction(i,lane_width=lane_width) for i in range(num_junctions)]

    roads = [road_classes.Road(road_lens[i],road_angles[i],i,lane_width=lane_width) for i in range(num_roads)]

    for i in range(num_roads):
        setJuncRoadJunc(junctions[junc_pairs[i][0]],roads[i],\
                              junctions[junc_pairs[i][1]])

    constructPhysicalOverlay(junctions)

    road_dict = {}
    lane,label = None,None
    for road in roads:
        lane = road.top_up_lane
        label = (int(lane.from_junction.label[1:]),int(lane.to_junction.label[1:]))
        road_dict[label]=road

    return junctions,road_dict
