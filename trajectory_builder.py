def computeWaypoint(stop_point,front_back_label):
    """Returns the coordinates of the midpoint of the specified side (front/back)
       of the stop_point."""
    fl = stop_point.four_corners["{}_left".format(front_back_label)]
    fr = stop_point.four_corners["{}_right".format(front_back_label)]
    pt = ((fl[0] + fr[0])/2,(fl[1]+fr[1])/2)
    return pt


def assignDijkstraScore(start_junc,cur_score,label):
    """Recursive algorithm that assigns a value to each lane in the map corresponding to the
       lane's distance from the initial start_junc. This value is then used in Dijkstra's method
       for shortest path identification."""
    # This uses Dijkstra's method to assign values to every lane on the map according to its
    # distance from the destination specified on the initial call of the function.
    len1 = None
    for lane in start_junc.in_lanes:
        #Erase remains from a previous trajectory generation
        #This is possible since runs of this routine cannot overlap
        len1 = len(label)
        if lane.dijkstra_score is not None and lane.dijkstra_score[-len1:] != label:
                lane.dijkstra_score = None

        #Finding shortest path. This shoukd also prevent recursion
        if lane.dijkstra_score is None or int(lane.dijkstra_score[:-len1])>cur_score+1:
            lane.dijkstra_score = str(cur_score+1)+label
            #We're traversing the path backwards, hence we go to the from_junction next
            assignDijkstraScore(lane.from_junction,cur_score+1,label)


def getShortestPath(start_point,end_point):
    """Given a start point (lane) and end point (junction) returns the trajectory
       (list of junctions) and waypoints (coordinates to aim for) of the shortest
       path linking the two points together."""
    next_stop = start_point.to_junction #next_stop is junction next to be entered
    trajectory = []
    waypoints = []
    pt = computeWaypoint(start_point,"front")
    trajectory.append(next_stop)
    waypoints.append(pt)
    poss_next = None
    candidate_list = None

    assignDijkstraScore(end_point,-1,end_point.label)

    label_len = len(end_point.label)
    cur_score = int(start_point.dijkstra_score[:-label_len])
    min_score,min_lane = None,None
    while cur_score>0:
        min_score = -1
        for lane in next_stop.out_lanes:
            if min_score == -1 or int(lane.dijkstra_score[:-label_len])<min_score:
                min_score = int(lane.dijkstra_score[:-label_len])
                min_lane = lane
        waypoints.append(computeWaypoint(min_lane,"front"))
        next_stop = min_lane.to_junction
        trajectory.append(min_lane.to_junction)
        cur_score = min_score
    return trajectory,waypoints


def buildTrajectory(start_point,end_point):
    """Returns the Trajectory (composed of junction objects) and the Waypoints
       (composed of the coordinates of the end of the appropriate lane that leads into
       the corresponding junction) joining the start and end points together"""
    trajectory,waypoints = getShortestPath(start_point,end_point)
    return trajectory,waypoints


def initialiseTrajectory(road,is_top_lane,destination):
    """Given the current road the vehicle is on (and boolean indicating lane on the road)
       intialises and returns a new trajectory with the next point the end of that lane"""
    #There is an argument to be made that this could be melted into buildTrajectory since
    #they are exclusively called in sequence. But for the time being I'll leave it as is.
    lane = None
    if is_top_lane:
        lane = road.top_up_lane
    else:
        lane = road.bottom_down_lane
    traj,waypoints = buildTrajectory(lane,destination)
    return traj,waypoints
