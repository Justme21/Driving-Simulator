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
        self.height = 2*lane_width

        #NOTE: This is where I stopped on Saturday 13th Jan.
        # Next step is to include into map builder the ability to resize the junction
        # from default when incoming our outgoing lane is at an angle to the junction
        self.x = None
        self.y = None

        #on is a list of all objects currently on the junction
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using print_contents in
        #      map_builder.py
        self.label = "J{}".format(label)


    def update_coords(self,x,y,road=None):
        """Updates (x,y), the location of the top left corner of the junction to be the 
           parameters x,y respectively. In map_builder it is an update to a junction that
           begins the process of anchoring the graph
           in map_builder.construct_physical_overlay.In an attempt to prevent looping we
           do not update the position of the road whose update resulted in the junction
           being updated (if it exists)."""
        #Once the wave of updates reaches a junction that is already in the correct
        # position it stops.This is, arguably, sloppy craftsmanship, but it works.
        # C'est la vie 
        if self.x != x or self.y != y:
            # The inputs are the intended locations for the junction's top left corner
            self.x = x
            self.y = y
            for lane in self.out_lanes:
                #Don't propogate update wave to road that we know just updated
                if lane.road is not road:    
                    lane.road.update_coords(self.x,self.y,self,lane)


    def print_status(self,mod=""):
        print("{}{}\tIN: {}\tOUT: {}\tWIDTH: {}\tHEIGHT: {}\t({},{})".format(mod,\
               self.label,[entry.label for entry in self.in_lanes],\
               [entry.label for entry in self.out_lanes],\
               round(self.width,2),round(self.height,2),\
               round(self.x,2),round(self.y,2)))


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
        self.top_up_lane.twin_lanes(self.bottom_down_lane)

        #The coordinates of the top_left corner of the road obect. Also the top left
        # corner of the top_up_lane. These values are set in 
        # map_builder.construct_physical_overlay
        self.x = None
        self.y = None

        #Length of the road.
        #NOTE: In the long run these values might not be needed for the road class.
        #      for the time being they remain until further testing can be done
        self.length = length
        self.width = 2*lane_width
        
        #on is a list of all objects currently on the junction
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using map_builder.print_contents
        self.label = "R{}".format(label)


    def update_coords(self,x,y,junction,lane):
        """Integral update function as it does most of the heavy lifting for updates.
           Given the coordinates of the previous junction determines where the top left
           for the road should be. This depends on the direction the road is going and
           whether it is going into or coming out of a junction.
           For convenience it also computes the coordinates for the alternate junction
           and passes those values to the corresponding update_coords function."""
        x_val, y_val = 0,0 
        next_junc = None
        direction = lane.direction
        
        #Used to identify which junction to send update wave to next
        if junction is lane.to_junction: next_junc = lane.from_junction
        else: next_junc = lane.to_junction        

        #Road is under the last junction
        if direction in range(226,315):
            x_val = round(x,2)
            y_val = round(y + junction.height,2)
            #Only propogate update if the coordinates here needed updating.
            #Otherwise thi road was updated before. 
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x+self.length*math.cos(math.radians(\
                                    direction)),self.y-self.length*math.sin(\
                                    math.radians(direction)),self)
        
        #Road is to the left of the previous junction
        elif direction in range(135,226):
            x_val = round(x + self.length*math.cos(math.radians(direction)),2)
            y_val = round(y - self.length*math.sin(math.radians(direction)),2)
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x-next_junc.width,self.y,self)
       
        #Road is above previous junction 
        elif direction in range(46,135):
            x_val = round(x + self.length*math.cos(math.radians(direction)),2)
            y_val = round(y - self.length*math.sin(math.radians(direction)),2)
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x,self.y-next_junc.height,self)
        
        #Road is to the right of previous junction (by process of elimination)
        else:
            x_val = round(x + junction.width,2)
            y_val = round(y,2)
            
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x+self.length*math.cos(math.radians(\
                                        direction)),self.y,self)

        #It does not matter when the lane's coordinates get updated, so leave to end 
        self.top_up_lane.update_coords()
        self.bottom_down_lane.update_coords()


    def print_status(self,mod=""):
        print("{}{}\tLEN: {}\tLANES: ({},{})\t({},{})\n".format(mod,\
                  self.label,round(self.length,2), self.top_up_lane.label,\
                  self.bottom_down_lane.label,round(self.x,2),round(self.y,2)))        


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

        #A reference to the other lane on this lane's road
        self.lane_twin = None

        #The road that this is a lane on
        self.road = road
        
        #Determines if the lane is going left/up, used in update_coords
        self.is_top_up = is_top_up


        #on is a list of all objects currently on the junction
        # Might be useful
        self.on = []

        #NOTE: Included to test structure construction using map_builder.print_contents
        self.label = "L{}".format(label)


    def twin_lanes(self,lane):
        """The parameter 'lane' is a reference to the other lane on the same 
           road as the ego-lane. This function provides each with a reference to the
           other for convenience in-program"""
        if self.is_top_up: #One lane can update both.
            self.lane_twin = lane
            lane.lane_twin = self


    def update_coords(self):
        """Update the coordinates of the lane. If the road is top lane (left to right)
           or the lane going up (if road is up/down) then the coordinates are just that
           of the road. Otherwise you must adjust it slightly to account for the width
           of the twinned road."""
        self.x = self.road.x
        self.y = self.road.y
        if not self.is_top_up:
            if (self.direction<=45 or self.direction>=315) or\
                         (self.direction in range(135,226)):
                # I can't imagine why the lanes wouldn't have the same width, but just
                # to be sure
                self.y+=self.lane_twin.width
            else:
                self.x+=self.lane_twin.width


    def print_status(self,mod=""):
        print("{}{}\tLEN: {}\tDIREC: {}\tFROM: {}\t TO: {}\t({},{})\n".format(mod,\
                  self.label,round(self.length,2),self.direction,\
                  self.from_junction.label, self.to_junction.label,\
                  round(self.x,2),round(self.y,2))) 
