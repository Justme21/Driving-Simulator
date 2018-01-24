import math

#NOTE: All computations done involving the y-coordinate here and in the simulator in
#      general must be inverted since it is designed to be compatible with pygame. 
#      i.e. in pygame (0,0) is in the top left of the screen. Thus increasing y sends
#      the point down the screen, and decreasing it sends it up 

#Width of a standard lane in the UK as found at
# https://en.wikipedia.org/wiki/Lane#Lane_width  . Unites are metres
lane_width = 3.7

class Junction():
    """In the symbolic network the Junctions play a crucial role as the nodes of the 
       graph holding the network together.
       The junctions are used to anchor the symbolic graph to a real-cartesian map.
       Juntions provide an avenue to get from one road to another.
       By design Junctions always have horizontal and vertical boundaries, though the
       lengths of the sides depends on the roads attached"""
    def __init__(self,label):
        self.in_lanes = [] #lanes leading into the junction. Might not be necessary.
        self.out_lanes = [] #lanes leading out of junction. Used to choose exit once in.

        #Junction dimensions get re-defined in map_builder.set_dimensions.
        # We use generic values on initialisation so that the junction will have
        # some dimension even if no lanes enter on that side
        self.width = 2*lane_width
        self.length = 2*lane_width

        #These coordinates get specified in updateCoords
        self.x = None
        self.y = None

        #The location of the four corners of the junction
        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}
        
        #This is mainly for ease of determining if a car has left whatever it is on
        #NOTE: If the value of self.direction is ever changed from 90 revise updteCoords
        self.direction = 90

        #on is a list of all objects currently on the junction
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using print_contents in
        #      map_builder.py
        self.label = "J{}".format(label)


    def updateCoords(self,corner,coord,in_road=None):
        """Updates (x,y), the location of the top left corner of the junction to be the 
           parameters x,y respectively. In map_builder it is an update to a junction that
           begins the process of anchoring the graph
           in map_builder.construct_physical_overlay.In an attempt to prevent looping we
           do not update the position of the road whose update resulted in the junction
           being updated (if it exists)."""
        #Once the wave of updates reaches a junction that is already in the correct
        # position it stops.This is, arguably, sloppy craftsmanship, but it works.
        # C'est la vie 
        cur_coords = self.four_corners[corner]
  
        if cur_coords is None or round(cur_coords[0],2) != round(coord[0],2) or \
           round(cur_coords[1],2) != round(coord[1],2):
            # The inputs are the intended locations for the junction's top left corner
            #This is put in just in case the four_corners get specified before update
            # is complete
            for x in self.four_corners: self.four_corners[x] = None
            self.four_corners[corner] = list(coord)
            setFourCorners(self)
            for lane in self.out_lanes:
                road = lane.road
                #Don't propogate update wave to road that we know just updated
                #This reasoning ONLY WORKS if self.direction=90
                if road is not in_road:
                    if lane.direction in range(0,46):
                        road.updateCoords("back_right",self.four_corners["front_left"],\
                                           self,lane)
                    elif lane.direction in range(46,90):
                        road.updateCoords("back_left",self.four_corners["front_left"]\
                                          ,self,lane)
                    elif lane.direction in range(90,135):
                        road.updateCoords("back_right",self.four_corners["front_right"]\
                                          ,self,lane)
                    elif lane.direction in range(135,180):
                        road.updateCoords("back_left",self.four_corners["front_right"]\
                                          ,self,lane)
                    elif lane.direction in range(180,226):
                        road.updateCoords("back_right",self.four_corners["back_right"]\
                                          ,self,lane)
                    elif lane.direction in range(226,270):
                        road.updateCoords("back_left",self.four_corners["back_right"]\
                                          ,self,lane)
                    elif lane.direction in range(270,315):
                        road.updateCoords("back_right",self.four_corners["back_left"]\
                                          ,self,lane)
                    else:
                        road.updateCoords("back_left",self.four_corners["back_left"]\
                                          ,self,lane)


    def putOnObject(self,obj):
        self.on.append(obj)


    def printStatus(self,mod=""):
        dims = ""
        for x in self.four_corners:
            dims += "({},{})\t".format(self.four_corners[x][0],self.four_corners[x][1])
        print("{}{}\tIN: {}\tOUT: {}\tWIDTH: {}\tHEIGHT: {}".format(mod,\
               self.label,[entry.label for entry in self.in_lanes],\
               [entry.label for entry in self.out_lanes],\
               round(self.width,2),round(self.length,2)))
        print("{}{}\t{}".format(mod,self.label,dims))


class Road():
    """Roads serve little function beyond encapsulating lanes - providing common
       direction and facilitating the construction of lanes in pairs."""
    def __init__(self,length,angle,label):
        #Top and Bottom when imagining road running from left to right.
        # Up/Down lanes are left to right rotated 90 degrees anti-clockwise (Top==Up)
        if angle>90 and angle <=270: angle = (180+angle)%360 #Ensures top lane is always going right/up
        self.top_up_lane = Lane(angle,length,label,self,1)
        self.bottom_down_lane = Lane((180+angle)%360,length,label,self,0)

        #We twin the lanes so that we can easily get from the top lane to the bottom
        # in program without having to go through the road.
        #The lane calling twinLanes must be the top lane (going left) or the lane
        # going up
        self.top_up_lane.twinLanes(self.bottom_down_lane)

        #The coordinates of the top_left corner of the road obect. Also the top left
        # corner of the top_up_lane. These values are set in 
        # map_builder.construct_physical_overlay
        self.x = None
        self.y = None

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


    def updateCoords(self,corner,coords,junction,lane):
        """Integral update function as it does most of the heavy lifting for updates.
           Given the coordinates of the previous junction determines where the top left
           for the road should be. This depends on the direction the road is going and
           whether it is going into or coming out of a junction.
           For convenience it also computes the coordinates for the alternate junction
           and passes those values to the corresponding updateCoords function."""
        x_val, y_val = 0,0 
        next_junc = None
        next_is_to = True
        direction = lane.direction
        
        #Used to identify which junction to send update wave to next
        if junction is lane.to_junction: 
            next_junc = lane.from_junction
            next_is_to = False
        else: next_junc = lane.to_junction        

        cur_coord = self.four_corners[corner]
        if cur_coord is None or (round(cur_coord[0],2) != round(coords[0],2)) or\
           (round(cur_coord[1],2) != round(coords[1],2)):
            self.four_corners[corner] = list(coords)
            setFourCorners(self)
            
            if direction in range(0,46):
                if next_is_to: tag = ["back_right","front_left"]
                else: tag = ["front_left","back_right"]
            elif lane.direction in range(46,90):
                if next_is_to: tag = ["front_left","back_right"]
                else: tag = ["back_right","back_left"]
            elif lane.direction in range(90,135):
                if next_is_to: tag = ["front_right","front_left"]
                else: tag = ["back_left","back_right"]
            elif lane.direction in range(135,180):
                if next_is_to: tag = ["back_left","front_right"]
                else: tag = ["front_right","back_left"]
            elif lane.direction in range(180,226):
                if next_is_to: tag = ["front_left","front_left"]
                else: tag = ["back_right","back_right"]
            elif lane.direction in range(226,270):
                if next_is_to: tag = ["back_right","front_right"]
                else: tag = ["front_left","back_left"]
            elif lane.direction in range(270,315):
                if next_is_to: tag = ["back_left","front_left"]
                else: tag = ["front_right","back_right"]
            else:
                if next_is_to: tag = ["front_right","front_right"]
                else: tag = ["back_left","back_left"]
            next_junc.updateCoords(tag[0],self.four_corners[tag[1]],self)

        """#Road is under the last junction
        if direction in range(226,315):
            x_val = round(x,2)
            y_val = round(y + junction.height,2)
            #Only propogate update if the coordinates here needed updating.
            #Otherwise thi road was updated before. 
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.updateCoords(self.x+self.length*math.cos(math.radians(\
                                    direction)),self.y-self.length*math.sin(\
                                    math.radians(direction)),self)
        
        #Road is to the left of the previous junction
        elif direction in range(135,226):
            x_val = round(x + self.length*math.cos(math.radians(direction)),2)
            y_val = round(y - self.length*math.sin(math.radians(direction)),2)
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.updateCoords(self.x-next_junc.width,self.y,self)
       
        #Road is above previous junction 
        elif direction in range(46,135):
            x_val = round(x + self.length*math.cos(math.radians(direction)),2)
            y_val = round(y - self.length*math.sin(math.radians(direction)),2)
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.updateCoords(self.x,self.y-next_junc.height,self)
        
        #Road is to the right of previous junction (by process of elimination)
        else:
            x_val = round(x + junction.width,2)
            y_val = round(y,2)
            
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.updateCoords(self.x+self.length*math.cos(math.radians(\
                                        direction)),self.y,self)
        """
        #It does not matter when the lane's coordinates get updated, so leave to end 
        self.top_up_lane.updateCoords()
        self.bottom_down_lane.updateCoords()


    def putOnObject(self,obj):
        """Adds a reference to an object to the list of objects 'on' the road"""
        self.on.append(obj)


    def printStatus(self,mod=""):
        dims = ""
        for x in self.four_corners:
            dims += "({},{})\t".format(self.four_corners[x][0],self.four_corners[x][1])
        print("{}{}\tLEN: {}\tLANES: ({},{})".format(mod,\
                  self.label,round(self.length,2), self.top_up_lane.label,\
                  self.bottom_down_lane.label))
        print("{}{}\t{}\n".format(mod,self.label,dims))


class Lane():
    def __init__(self,direction,length,label,road,is_top_up):
        #The direction the lane is going relative to 0 degrees (horizontal).
        self.direction = direction

        #Fairly self-explanatory.
        self.length = length 
        self.width = lane_width

        #From and to based on left (top on horizontal road) lane of road.
        self.from_junction = None
        self.to_junction = None
        
        #Each lane has it's own physical dimensions to make it easier to determine which
        # lane the vehicle is in. 
        self.x = None
        self.y = None

        self.four_corners = {"front_left":None,"front_right":None,"back_left":None,\
                             "back_right":None}
        
        #A reference to the other lane on this lane's road
        self.lane_twin = None

        #The road that this is a lane on
        self.road = road
        
        #Determines if the lane is going left/up, used in updateCoords
        self.is_top_up = is_top_up


        #on is a list of all objects currently on the junction
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using map_builder.print_contents
        self.label = "L{}".format(label)


    def twinLanes(self,lane):
        """The parameter 'lane' is a reference to the other lane on the same 
           road as the ego-lane. This function provides each with a reference to the
           other for convenience in-program"""
        if self.is_top_up: #One lane can update both.
            self.lane_twin = lane
            lane.lane_twin = self


    def putOnObject(self,obj):
        self.on.append(obj)
        self.road.putOnObject(obj)


    def updateCoords(self):
        """Update the coordinates of the lane. If the road is top lane (left to right)
           or the lane going up (if road is up/down) then the coordinates are just that
           of the road. Otherwise you must adjust it slightly to account for the width
           of the twinned road."""
        if self.is_top_up:
            self.four_corners["front_left"] = list(self.road.four_corners["front_left"])
        else:
            self.four_corners["front_right"] = list(self.road.four_corners["front_right"])

        setFourCorners(self)

        """self.x = self.road.x
        self.y = self.road.y
        if not self.is_top_up:
            if (self.direction<=45 or self.direction>=315) or\
                         (self.direction in range(135,226)):
                # I can't imagine why the lanes wouldn't have the same width, but just
                # to be sure
                self.y+=self.lane_twin.width
            else:
                self.x+=self.lane_twin.width
        """

    def printStatus(self,mod=""):
        dims = ""
        for x in self.four_corners:
            dims += "({},{})\t".format(self.four_corners[x][0],self.four_corners[x][1])
        print("{}{}\tLEN: {}\tDIREC: {}\tFROM: {}\t TO: {}".format(mod,\
                  self.label,round(self.length,2),self.direction,\
                  self.from_junction.label, self.to_junction.label)) 
        print("{}{}\t{}\n".format(mod,self.label,dims))


def setFourCorners(obj):
    #There should only be one entry in this list
    start = [x for x in obj.four_corners if obj.four_corners[x] is not None][0]
    order = {"back_right":"front_right","front_right":"front_left",\
             "front_left":"back_left","back_left":"back_right"}
    end = str(start)
    pt_init = None
    direc,disp = None,None
    while start is not end or direc is None:
        if start is "back_right":
            direc = math.radians(obj.direction)
            disp = obj.length
        elif start is "front_right":
            direc = math.radians(obj.direction+90)
            disp = obj.width
        elif start is "front_left":
            direc = math.radians(obj.direction+180)
            disp = obj.length
        elif start is "back_left":
            direc = math.radians(obj.direction+270)
            disp = obj.length

        pt_init = obj.four_corners[start]
        print("PT: {}\tORDER: {}".format(pt_init,order[start]))
        obj.four_corners[order[start]] = [round(pt_init[0] + disp*math.cos(direc),2), \
                                          round(pt_init[1] + disp*math.sin(direc),2)]
        start = order[start]
