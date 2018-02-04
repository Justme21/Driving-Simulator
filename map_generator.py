import random

#random.seed(493072)

REVERSE_DICT = {"top":"bottom","bottom":"top","left":"right","right":"left"}

class Graph():
    def __init__(self):
        self.coords = []
        self.node_list = []
        self.link_list = []
        self.num_links = 0
        self.num_nodes = 1
        node = Node(0,0,0)
        self.node_list.append(node)
        self.coords.append((0,0))


    def addLink(self,link,node_from,node_to,direc):
        link.attachToNodes(node_from,node_to,direc)
        self.link_list.append(link)
        self.num_links += 1


    def addNode(self,node):
        self.num_nodes += 1
        self.coords.append(node.coordinates)
        self.node_list.append(node)


    def buildGraph(self,angle_dict,node,node_limit,link_limit,angle_constr=[]):
        open_direc = [x for x in node.directions if node.directions[x] is None]
        print("LABEL: {}\tAVAIL: {}".format(node.label,open_direc))
        angle = None
        num_links = random.randint(1,min(link_limit-self.num_links,len(open_direc)))
        print("TRYING TO PRINT {} LINKS".format(num_links))
        node_coords = self.coords[self.node_list.index(node)]
        new_links = []
        new_angles = []
        for i in range(num_links):
            angle,direc = getAngle(open_direc,angle_constr,angle_dict)
            new_angles.append(int(angle))
            link = Link(angle)
            fail = True
            new_coords = list(node_coords)
            for i in range(2): new_coords[i] += link.change[i]
            if new_coords in self.coords:
                new_node = self.node_list[self.coords.index(new_coords)]
                if new_node.directions[REVERSE_DICT[direc]] is None:
                    self.addLink(link,node,new_node,direc)
                    angle_dict[direc].remove(angle)
                    open_direc.remove(direc)
                    new_links.append(link)
                    fail = False
            else:
                if self.num_nodes<node_limit:
                    new_node = Node(new_coords[0],new_coords[1],self.num_nodes)
                    self.addNode(new_node)
                    self.addLink(link,node,new_node,direc)
                    new_links.append(link)
                    angle_dict[direc].remove(angle)
                    open_direc.remove(direc)
                    print("Created New Node: {}".format(direc))
                    fail = False
        if fail:
            print("Tried these angles: {} and failed".format(new_angles))
        else:
            print("Succeeded in creating a link: {}".format(direc))
        #exit(-1)
        if fail:
            print("Still in same node")
            for entry in new_angles: angle_constr.append(entry)
            self.buildGraph(angle_dict,node,node_limit,link_limit,angle_constr)
        else:
            print("Going onto other nodes")
            for link in new_links:
                if self.num_links<link_limit:
                    self.buildGraph(angle_dict,link.to_node,node_limit,link_limit)
                else:
                    #This is the end condition for the loop that does not exit out of the
                    # program
                    pass
                      
 
    def printStatus(self):
        print("NODES: {}\nLINKS: {}".format(self.num_nodes,self.num_links))
        for entry in self.link_list:
            print("OUT: {}\tANGLE: {}\tIN: {}".format(entry.from_node.label,entry.angle,\
                                                entry.to_node.label))


class Node():
    def __init__(self,i,j,k):
        self.label = k
        self.coordinates = (i,j)
        self.directions = {"top":None,"bottom":None,"left":None,"right":None}


class Link():
    def __init__(self,angle):
        self.angle = angle
        self.from_node = None
        self.to_node = None

        if angle in range(46,135): self.change = [0,1]
        elif angle in range(135,226): self.change = [-1,0]
        elif angle in range(226,315): self.change = [0,-1]
        else: self.change = [1,0]


    def attachToNodes(self,node_from,node_to,from_direc):
        node_from.directions[from_direc] = self
        node_to.directions[REVERSE_DICT[from_direc]] = self
        self.from_node = node_from
        self.to_node = node_to


def getAngle(open_direc,angle_constr,angle_dict):
    pos = 0
    angle = None
    angle_list = None
    open_direc = list(open_direc)
    print("AVAIL: {}".format(open_direc))
    print("ANGLES: {}".format(angle_dict))
    print("EXCLUDING: {}".format(angle_constr))
    while angle is None:
        direc = random.choice(open_direc)
        pos = 0
        if len(angle_dict[direc])>0:
            angle_list = angle_dict[direc]
            while pos<len(angle_list) and angle_list[pos] in angle_constr: pos += 1
            if pos == len(angle_list): open_direc.remove(direc)
            else: angle = angle_dict[direc][pos]
        else: open_direc.remove(direc)
        if open_direc == []:
            print("ERROR; Not Possible to Construct Graph")
            exit(-1)
    return angle,direc


def partitionAngles(angles):
    direction_dict = {"top":[],"bottom":[],"left":[],"right":[]}
    label = None
    for entry in angles:
        if entry in range(46,135):
            label = "top"
        elif entry in range(135,226):
            label = "left"
        elif entry in range(226,315):
            label = "bottom"
        else:
            label = "right"
        direction_dict[label].append(entry)
    return direction_dict
        

def angleTest(angles,num_junctions,num_roads):
    top_bottom = [x for x in angles if x in range(46,135) or x in range(226,315)]
    left_right = [x for x in angles if x not in top_bottom]
    if len(top_bottom)>2*num_junctions:
        return -1
    elif len(left_right)>2*num_junctions:
        return 1
    return 0


def probAngleSetter(num_angles,rnge):
    prob_angle_list = []
    for i in range(num_angles):
        prob_angle_list.append(random.randint(rnge[0],rnge[1]))
    return prob_angle_list

def randomiseAngles(count,num_junctions,ang_list):
    randomiser = None
    for i,entry in enumerate(ang_list):
        randomiser = random.choice([True,False])
        if count<num_junctions and randomiser:
            ang_list[i] = (entry+180)%360
            count += 1
    return ang_list,count


def generateAngles(num_junctions,num_roads):
    num_up = None    
    angles = []
    count = None

    while num_up is None or num_up > 2*num_junctions:
        num_up = random.randint(0,num_roads)

    up_angles = [x for x in probAngleSetter(num_up,(46,135))]
    left_angles = [x for x in probAngleSetter(num_roads-num_up,(135,226))]

    for ang_list in [up_angles,left_angles]:
        count = 0
        ang_list,count = randomiseAngles(count,num_junctions,ang_list)
        while len(ang_list)-count>num_junctions:
            ang_list,count = randomiseAngles(count,num_junctions,ang_list)
        if count>num_junctions and len(ang_list)-count>num_junctions:
                print("That's some pretty backward reasoning bro")
                exit(-1) 
        angles += ang_list
    return angles


num_junctions = 6
num_roads = 7

angles = None

if angles is None or len(angles)!= num_roads or  angleTest(angles,num_junctions,num_roads)!=0:
    angles = generateAngles(num_junctions,num_roads)

sorted_angles = partitionAngles(angles)
temp = None
for entry in sorted_angles:
    while len(sorted_angles[entry])>num_junctions:
        temp = sorted_angles[entry][0]
        sorted_angles[entry].remove(temp)
        sorted_angles[REVERSE_DICT[entry]].append(temp)

graph = Graph()
graph.buildGraph(sorted_angles,graph.node_list[0],num_junctions,num_roads)
graph.printStatus()
