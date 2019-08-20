import math

#NOTE: All computations done involving the y-coordinate here and in the simulator in
#      general must be inverted since it is designed to be compatible with pygame. 
#      i.e. in pygame (0,0) is in the top left of the screen. Thus increasing y sends
#      the point down the screen, and decreasing it sends it up 

#Width of a standard lane in the UK as found at
# https://en.wikipedia.org/wiki/Lane#Lane_width  . Unites are metres
LANE_WIDTH = 3.7

class Junction():
    """In the symbolic network the Junctions play a crucial role as the nodes of the
       graph holding the network together.
       The junctions are used to anchor the symbolic graph to a real-cartesian map.
       Juntions provide an avenue to get from one road to another.
       By design Junctions always have horizontal and vertical boundaries, though the
       lengths of the sides depends on the roads attached"""
    def __init__(self,label,lane_width=None):
        self.in_lanes = [] #lanes leading into the junction. Might not be necessary.
        self.out_lanes = [] #lanes leading out of junction. Used to choose exit once in.

        #Junction dimensions get re-defined in map_builder.set_dimensions.
        # We use generic values on initialisation so that the junction will have
        # some dimension even if no lanes enter on that side
        if lane_width is None:
            lane_width = LANE_WIDTH
        self.width = 2*lane_width
        self.length = 2*lane_width

        #The location of the four corners of the junction
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}

        #This is mainly for ease of determining if a car has left whatever it is on
        #NOTE: If the value of self.direction is changed from 90 revise updteCoords
        self.direction = 90

        #self.on is a list of all objects currently on the junction
        self.on = []

        #NOTE: Included to test structure construction using print_contents in
        #      map_builder.py
        self.label = "J{}".format(label)


    def updateCoords(self,coords,in_road=None):
        """Updates the location of the four corners of the junction such that the
           specified corner (corner) has the set coordinates (coords).
           If the update was called from a road then in_road is a reference to that road
           (this is used to omit road when the update is passed on to adjoined roads,
           preventing looping).
           In map_builder it is an update to a junction that begins the process of 
           anchoring the graph in map_builder.construct_physical_overlay."""

        if None in list(self.four_corners.values()):
            cur_coords = None
        else:
            cur_coords = [(self.four_corners["front_left"][0]+self.four_corners["front_right"][0])/2,\
                    (self.four_corners["front_left"][1]+self.four_corners["back_left"][1])/2]

        #Once the wave of updates reaches a junction that is already in the correct
        # position it stops.This is, arguably, sloppy craftsmanship, but it works.
        # C'est la vie  
        if cur_coords is None or math.sqrt((cur_coords[0]-coords[0])**2+(cur_coords[1]-coords[1])**2)>2:
        #if cur_coords is None or round(cur_coords[0],2) != round(coords[0],2) or \
        #   round(cur_coords[1],2) != round(coords[1],2):
            #Unset four_corners
            for x in self.four_corners: self.four_corners[x] = None
            setFourCorners(self,"front_left",[coords[0]-self.width/2,coords[1]-self.length/2])

            #Propogate update wave
            for lane in self.out_lanes:
                road = lane.road
                #Don't propogate update wave to road that we know just updated
                #This reasoning ONLY WORKS if self.direction=90
                if road is not in_road:
                    road.updateCoords(coords,self)


    def putOn(self,obj):
        """Adds an object (obj) to the list of things currently on the junction.
           For thoroughness also puts on obj.on if not already there."""
        if obj not in self.on:
            self.on.append(obj)
        if self not in obj.on:
            obj.putOn(self)


    def takeOff(self,obj):
        """Removes an object (obj) from the list of things currently on the junction.
           For thoroughness also takes from obj.on if still there"""
        if obj in self.on:
            self.on.remove(obj)
        if self in obj.on:
            obj.takeOff(self)


    def printStatus(self,mod=""):
        """Used to print properties of the junction during debugging"""
        dims = ""
        corner_labels = {"back_right":"br","back_left":"bl","front_right":"fr",\
                         "front_left":"fl"}
        for x in self.four_corners:
            dims += "{}({},{}), ".format(corner_labels[x],self.four_corners[x][0],\
                                         self.four_corners[x][1])
        print("{}{}\tIN: {}\tOUT: {}\tWIDTH: {}\tHEIGHT: {}".format(mod,\
               self.label,[entry.label for entry in self.in_lanes],\
               [entry.label for entry in self.out_lanes],\
               round(self.width,2),round(self.length,2)))
        print("{}{}\t{}".format(mod,self.label,dims))


class Road():
    """Roads serve little function beyond encapsulating lanes - providing common
       direction and facilitating the construction of lanes in pairs."""
    def __init__(self,length,angle,label,lane_width=None):
        if lane_width is None:
            lane_width = LANE_WIDTH

        #Top and Bottom when imagining road running from left to right.
        # Up/Down lanes are left to right rotated 90 degrees anti-clockwise (Top==Up)
        if angle>90 and angle <=270: angle = (180+angle)%360 #Ensures top lane is always going right/up
        self.top_up_lane = Lane(angle,length,lane_width,label,self,1)
        self.bottom_down_lane = Lane((180+angle)%360,length,lane_width,label,self,0)

        #We twin the lanes so that we can easily get from the top lane to the bottom
        # in program without having to go through the road.
        #The lane calling twinLanes must be the top lane (going left) or the lane
        # going up
        self.top_up_lane.twinLanes(self.bottom_down_lane)

        #The location of the four corners of the junction
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}

        #The direction of the road is the angle of the top_up_lane (i.e.right and up)
        self.direction = angle

        #Length of the road.
        #NOTE: In the long run these values might not be needed for the road class.
        #      for the time being they remain until further testing can be done
        self.length = length
        self.width = 2*lane_width

        #on is a list of all objects currently on the road
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using map_builder.print_contents
        self.label = "R{}".format(label)


    def updateCoords(self,coords,junction):
        """Integral update function as it does most of the heavy lifting for updates.
           Given the coordinates (coords) for where a specified corner of the road 
           (corner) determines where the specified corner (corner) of the road should be.
           Reference to the junction that called the update is passed so that that
           junction's coordinates are not updated, preventing looping"""
        next_junc = None
        next_is_to = True
        direction = self.direction

        #Used to identify which junction to send update wave to next
        if junction is self.top_up_lane.to_junction:
            next_junc = self.top_up_lane.from_junction
            next_is_to = False
            coord_tag = "front"
        else:
            next_junc = self.top_up_lane.to_junction
            coord_tag = "back"

        if None not in list(self.four_corners.values()):
            cur_coords = [(self.four_corners[coord_tag+"_left"][0]+self.four_corners[coord_tag+"_right"][0])/2,\
                    (self.four_corners[coord_tag+"_left"][1]+self.four_corners[coord_tag+"_right"][1])/2]
        else:
            cur_coords = None

        if cur_coords is None or math.sqrt((cur_coords[0]-coords[0])**2 + (cur_coords[1]-coords[1])**2)>2:
        #if cur_coord is None or (round(cur_coord[0],2) != round(coords[0],2)) or\
        #   (round(cur_coord[1],2) != round(coords[1],2)):
            width = self.width/2
            length = self.length
            setFourCorners(self,coord_tag+"_right",[coords[0]+width*math.cos(math.radians(direction-90)),\
                    coords[1]-width*math.sin(math.radians(direction-90))])

            if next_is_to:
                next_junc.updateCoords([coords[0]+length*math.cos(math.radians(direction)),\
                        coords[1]-length*math.sin(math.radians(direction))],self)
            else:
                next_junc.updateCoords([coords[0]-length*math.cos(math.radians(direction)),\
                        coords[1]+length*math.sin(math.radians(direction))],self)

        #It does not matter when the lane's coordinates get updated, so leave to end 
        self.top_up_lane.updateCoords("front_left",self.four_corners["front_left"])
        self.bottom_down_lane.updateCoords("front_left",self.four_corners["back_right"])


    def putOn(self,obj):
        """Adds a reference to an object to the list of objects 'on' the road.
           Also adds self to obj.on if not already there."""
        if obj not in self.on:
            self.on.append(obj)
        if self not in obj.on:
            obj.putOn(self)


    def takeOff(self,obj):
        """Removes reference to object from self.on. Also removes self from obj.on
           if not already done."""
        if obj in self.on:
            self.on.remove(obj)
        if self in obj.on:
            obj.takeOff(self)


    def printStatus(self,mod=""):
        """Used to print the properties of the road during debugging"""
        dims = ""
        corner_labels = {"back_right":"br","back_left":"bl","front_right":"fr",\
                         "front_left":"fl"}
        for x in self.four_corners:
            dims += "{}({},{}), ".format(corner_labels[x],self.four_corners[x][0],\
                                         self.four_corners[x][1])
        print("{}{}\tLEN: {}\tLANES: ({},{})".format(mod,\
                  self.label,round(self.length,2), self.top_up_lane.label,\
                  self.bottom_down_lane.label))
        print("{}{}\t{}\n".format(mod,self.label,dims))


class Lane():
    def __init__(self,direction,length,width,label,road,is_top_up):
        #The direction the lane is going relative to 0 degrees (horizontal).
        self.direction = direction

        #Fairly self-explanatory.
        self.length = length
        self.width = width

        #From and to based on left (top on horizontal road) lane of road.
        self.from_junction = None
        self.to_junction = None

        #Dictionary to store the locations of the four corners of the lane
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}

        #A reference to the other lane on this lane's road
        self.lane_twin = None

        #The road that this is a lane on
        self.road = road

        #Determines if the lane is going left/up, used in updateCoords
        self.is_top_up = is_top_up


        #on is a list of all objects currently on the junction
        self.on = []

        #NOTE: Included to test structure construction using map_builder.print_contents
        if is_top_up: label = str(label) + "T"
        else: label = str(label) + "B"
        self.label = "L{}".format(label)
        self.dijkstra_score = None


    def twinLanes(self,lane):
        """The parameter 'lane' is a reference to the other lane on the same
           road as the ego-lane. This function provides each with a reference to the
           other for convenience in-program"""
        if self.is_top_up: #One lane can update both.
            self.lane_twin = lane
            lane.lane_twin = self


    def putOn(self,obj):
        """Add the specified object to the list of objects on the lane.
           Also add it onto the road if it is not already there"""
        if obj not in self.on:
            self.on.append(obj)
        if self not in obj.on:
            obj.putOn(self)
        if obj not in self.road.on:
            self.road.putOn(obj)


    def takeOff(self,obj):
        """Removes obj from self.on if it is there. Also removes self from
           obj.on if it has not already been removed"""
        if obj in self.on:
            self.on.remove(obj)
        if self in obj.on:
            obj.takeOff(self)

    def updateCoords(self,corner,coords):
        """Update the coordinates for the corners of the lane by calling
           setFourCorners."""
        #Unset the four corners if they have been previously set. 
        #Since this is only called if the road coordinates have updated
        # there is no reason to check if the values are correct
        if self.four_corners[corner] is not None:
            for entry in self.four_corners:
                self.four_corners[entry] = None
        setFourCorners(self,corner,coords)


    def printStatus(self,mod=""):
        """Prints properties of the lane during debugging"""
        dims = ""
        corner_labels = {"back_right":"br","back_left":"bl","front_right":"fr",\
                         "front_left":"fl"}
        for x in self.four_corners:
            dims += "{}({},{}), ".format(corner_labels[x],self.four_corners[x][0],\
                                         self.four_corners[x][1])
        print("{}{}\tLEN: {}\tDIREC: {}\tFROM: {}\t TO: {}".format(mod,\
                  self.label,round(self.length,2),self.direction,\
                  self.from_junction.label, self.to_junction.label)) 
        print("{}{}\t{}\n".format(mod,self.label,dims))


def setFourCorners(obj,corner,coords):
    """Given an object from road_classes this assigns values to each of the entries in
       the dictionary four_corners (element of each object). Before this function
       is called one of the values for four_corners is set. This function then 
       cycles through the remaining corners in order so that each value can be 
       determined from those set immediately before"""

    obj.four_corners[corner] = list(coords)
    order = {"back_right":"front_right","front_right":"front_left",\
             "front_left":"back_left","back_left":"back_right"}
    start = corner
    #Copy start to know when we have travelled all the points on the 
    # edge of the shape
    end = str(start)
    pt_init = None
    direc,disp = None,None
    #Iterate through all the corners until there are none left that are not set
    while start != end or direc is None:
        if start == "back_right":
            direc = math.radians(obj.direction)
            disp = obj.length
        elif start == "front_right":
            direc = math.radians(obj.direction+90)
            disp = obj.width
        elif start == "front_left":
            direc = math.radians(obj.direction+180)
            disp = obj.length
        elif start == "back_left":
            direc = math.radians(obj.direction+270)
            disp = obj.width

        pt_init = obj.four_corners[start]
        obj.four_corners[order[start]] = [round(pt_init[0] + disp*math.cos(direc),2), \
                                          round(pt_init[1] - disp*math.sin(direc),2)]

        start = order[start]
