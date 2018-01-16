import math

lane_width = 3.7


class Junction():
    def __init__(self,label):
        self.out_lanes = [] #lanes leading into the junction. Might not be necessary.
        self.in_lanes = [] #lanes leading out of junction. Used to choose exit once in.

        self.num_lanes = None
        
        #NOTE: Junction dimensions need to be defined on construction.
        # We use generic values on initialisation so that the junction will have
        # some dimension even if no lanes enter on that side
        self.width = 2*lane_width
        self.height = 2*lane_width

        #NOTE: This is where I stopped on Saturday 13th Jan.
        # Next step is to include into map builder the ability to resize the junction
        # from default when incoming our outgoing lane is at an angle to the junction
        self.x = None
        self.y = None

        #NOTE: Included to test structure construction using print_contents in map_builder.py
        self.label_TEMP = label

    def update_coords(self,x,y,road=None):
        self.x = x
        self.y = y
        for lane in self.out_lanes:
            if lane.road is not road:    
                lane.road.update_coords(self.x,self.y,self,lane)


class Road():
    def __init__(self,length,angle,label):
        #Top and Bottom when imagining road running from left to right.
        # Up/Down lanes are left to right rotated 90 degrees anti-clockwise (Top==Up)
        if angle>90 and angle <=270: angle = (180+angle)%360 #Ensures top lane is always going right/up
        self.top_up_lane = Lane(angle,length,label,self,1)
        self.bottom_down_lane = Lane((180+angle)%360,length,label,self,0)

        self.top_up_lane.twin_lanes(self.bottom_down_lane)

        self.x = None
        self.y = None

        #Length of the road.
        #NOTE: Not sure whether this should be here or if it belongs in the lane class.
        #Return later.
        self.length = length
        self.width = 2*lane_width #NOTE: This is probably excessive but included just in case


        #NOTE: Included to test structure construction using print_contents in map_builder.py
        self.label_TEMP = label


    def update_coords(self,x,y,junction,lane):
        x_val, y_val = 0,0 
        next_junc = None
        if junction is lane.to_junction: next_junc = lane.from_junction
        else: next_junc = lane.to_junction
        
        if lane.direction in range(226,315):
            x_val = x
            y_val = y + junction.height
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x+self.length*math.cos(math.radians(\
                                    lane.direction)),self.y+self.length*math.sin(\
                                    math.radians(lane.direction)),self)
        elif lane.direction in range(135,226):
            x_val = x+self.length*math.cos(math.radians(lane.direction))
            y_val = y
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x-next_junc.width,self.y,self)
        elif lane.direction in range(46,135):
            x_val = x
            y_val = y - self.length*math.sin(math.radians(lane.direction))
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x,self.y- next_junc.heightself.length*\
                                        math.sin(math.radians(lane.direction)),self)
        else:
            x_val = x + junction.width
            y_val = y
            
            if self.x != x_val or self.y != y_val:
                self.x = x_val
                self.y = y_val
                next_junc.update_coords(self.x+self.length*math.cos(math.radians(\
                                        lane.direction)),self.y,self)
 
        self.top_up_lane.update_coords()
        self.bottom_down_lane.update_coords()
        

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

        self.is_top_up = is_top_up
        self.road = road
        #NOTE: Included to test structure construction using print_contents in map_builder.py
        self.label_TEMP = label

    def twin_lanes(self,lane):
        if self.is_top_up:
            self.lane_twin = lane
            lane.lane_twin = self

    def update_coords(self):
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
