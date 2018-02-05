import random

#random.seed(493072)

REVERSE_DICT = {"top":"bottom","bottom":"top","left":"right","right":"left"}

class Graph():
    def __init__(self):
        #A list storing the coordinates of all the nodes of the graph
        self.coords = []
        #List of all the nodes in the graph, corresponding to self.coords
        self.node_list = []
        #List of all links in the graph. Used to print output
        self.link_list = []
        #The number of links in the graph
        self.num_links = 0
        #The number of nodes in the graph
        self.num_nodes = 1
        
        #By default the graph's first node is generated with coordinates (0,0)
        node = Node(0,0,0)
        self.node_list.append(node)
        self.coords.append((0,0))


    def addLink(self,link,node_from,node_to,direc):
        """Connect link to node_from and node_to and adds the link to the 
           list of links on the graph, incrementing the number of links"""
        link.attachToNodes(node_from,node_to,direc)
        self.link_list.append(link)
        self.num_links += 1


    def addNode(self,node):
        """Add node to the list of nodes on the graph, adding coordinates to the list
           of coordinates, and incrementing the number of nodes"""
        self.num_nodes += 1
        self.coords.append(node.coordinates)
        self.node_list.append(node)


    def buildGraph(self,angle_dict,node,node_limit,link_limit,direc_constr=[]):
        """Recursive algorithm that randomly chooses angles from a precomputed list
           and randomly assigns them to links and goes about the process of constructing
           nodes and links to construct a viable graph network.
           angle_dict is a dictionary containing lists of angle labels that can be assigned,
           partitioned by direction (top,bottom,left,right)
           node is the current node that is being considered, that the links generated
           in this running will be attached to. 
           node_limit and link_limit are the specified maximum number of nodes and links 
           permitted to be on the graph.
           angle_constr contains the list of angles that cannot be used in constructing links
           (because they have been previously assigned to other links in this round)"""
        #The list of unattached directions on the current node
        open_direc = [x for x in node.directions if node.directions[x] is None]
        print("LABEL: {}\tAVAIL: {}".format(node.label,open_direc))
        angle = None
        #The number of links that will be attempted to be created this round
        num_links = random.randint(1,min(link_limit-self.num_links,len(open_direc)))
        print("TRYING TO PRINT {} LINKS".format(num_links))
        #Coordinates of the current node
        node_coords = self.coords[self.node_list.index(node)]
        new_links = []
        new_direcs = []
        for i in range(num_links):
            #getAngle chooses an angle from angle_dict that corresponds to an available side
            # of the current node. If getAngle fails to find any such angle it leaves angle
            # as none and sets direc to be the set of directions for which there are angles
            # yet unassigned
            angle,direc = getAngle(open_direc,direc_constr,angle_dict)
            #If angle is none then getAngle failed to find an angle that can be linked to
            # the node
            if angle is None:
                #Here we go over every node to find ones that still have openings on
                # sides corresponding to the angles we have
                open_nodes = []
                for entry in self.node_list:
                    #direc is now a list of directions corresponding to the actions available
                    for avail in direc:
                        # Only add the node to the list of nodes to be considered 
                        if avail in [x for x in entry.directions\
                               if entry.directions[x] is not None]:
                            open_nodes.append(entry)
                            #Don't need to iterate over all angles if node works for any one
                            break
                #Keep checking while there are still nodes to check and angle is not assigned
                while angle is None and open_nodes != []:
                    #These are the same steps that occur at the start of the program
                    node = random.choice(open_nodes)
                    node_coords = self.coords[self.node_list.index(node)]
                    open_direc = [x for x in node.directions if node.directions[x] is None]
                    angle,direc = getAngle(open_direc,[],angle_dict)
                if open_nodes == []:
                    print("ERROR: No Suitable Nodes Available. Terminating")
                    exit(-1)
            #By this point we have chosen a direction, we add it to the list of directions
            # being used in this round
            new_direcs.append(direc)
            #Create the angle being inserted this round
            link = Link(angle)
            fail = True
            #Copy the coordinates of the current node that will be modified to be the 
            # coordinates for the node the link will connect to 
            new_coords = list(node_coords)
            for i in range(2): new_coords[i] += link.change[i]
            #There already exists a node in the graph with the same coordinates as the 
            # node the link will connect to
            if new_coords in self.coords:
                #set the target node to the the corresponding entry in the list of nodes
                new_node = self.node_list[self.coords.index(new_coords)]
                #Identified target node is suitable for connection
                # In this case we add link to graph, remove angle from available angles
                # and remove direction from list of available directions on the node
                if new_node.directions[REVERSE_DICT[direc]] is None:
                    self.addLink(link,node,new_node,direc)
                    angle_dict[direc].remove(angle)
                    open_direc.remove(direc)
                    #Add this link to the list of links created this round
                    new_links.append(link)
                    #We have not failed to create a new link
                    fail = False
                #If the above clause does not trigger then we have failed to make a link
            else:
                #If a node with the same coordinates does not exist then the only
                # option is to create a new node. To do this we must check that we
                # are below the node limit
                if self.num_nodes<node_limit:
                    #Build a new node with the suitable coordinates
                    new_node = Node(new_coords[0],new_coords[1],self.num_nodes)
                    #Add the node and link to the graph
                    self.addNode(new_node)
                    self.addLink(link,node,new_node,direc)
                    #Add this link to the list of links created this round
                    new_links.append(link)
                    #Removre direction and angle from lists of consideration
                    angle_dict[direc].remove(angle)
                    open_direc.remove(direc)
                    print("Created New Node: {}".format(direc))
                    #We have not failed to create a new link
                    fail = False
                #If the above does not trigger we have faile to make a link
        if fail:
            print("Tried these directions: {} and failed".format(new_direcs))
        else:
            print("Succeeded in creating a link: {}".format(direc))
        #exit(-1)
        if fail:
            #If fail is true we have not made any new links this round
            print("Still in same node")
            #We remove the directions we have previously tried this round from consideration
            complete = True
            for entry in new_direcs: 
                if entry not in direc_constr:
                    direc_constr.append(entry)
                    complete = False
            if complete:
                print("There's something wrong here")
                #exit(-1)
            else:
                #Recursively try again building on the same node, but with the new constraints
                self.buildGraph(angle_dict,node,node_limit,link_limit,direc_constr)
        else:
            #We have successfully constructed at least one link. We move onto the new nodes
            # and try make new links from there (if we need to make more links)
            print("Going onto other nodes")
            for link in new_links:
                if self.num_links<link_limit:
                    #If more links to be made make them off the new connections
                    self.buildGraph(angle_dict,link.to_node,node_limit,link_limit)
                else:
                    #This is the end condition for the loop that does not exit out of the
                    # program
                    pass
                      
 
    def printStatus(self):
        """The graph can print it's status for debugging purposes (and also to inspect the
           connections created)"""
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
        #Link contains the angle as a label as well as references to nodes it links to
        self.angle = angle
        self.from_node = None
        self.to_node = None

        #change indicates how adjacent nodes coordinates should differ if 
        # connected by this link. Links can be bijectively mapped to the 4 sides of a 
        # square, thus no diagonals are required
        if angle in range(46,135): self.change = [0,1]
        elif angle in range(135,226): self.change = [-1,0]
        elif angle in range(226,315): self.change = [0,-1]
        else: self.change = [1,0]


    def attachToNodes(self,node_from,node_to,from_direc):
        """Given two nodes establishes this link as the connection between them"""
        node_from.directions[from_direc] = self
        node_to.directions[REVERSE_DICT[from_direc]] = self
        self.from_node = node_from
        self.to_node = node_to


def getAngle(open_direc,direc_constr,angle_dict):
    """Given a list of availabl directions, and the generated angles associated with each
       direction, returns an angle and direction that can be used to attempt to create a
       link"""
    angle = None
    angle_list = None
    open_direc = list(open_direc)
    open_direc = [x for x in open_direc if x not in direc_constr]
    print("DIREC AVAIL: {}".format(open_direc))
    print("ANGLES: {}".format(angle_dict))
    print("EXCLUDING: {}".format(direc_constr))
    while angle is None:
        if open_direc != []:
            direc = random.choice(open_direc)
            if len(angle_dict[direc])>0:
                angle = random.choice(angle_dict[direc])
            else: open_direc.remove(direc)
        if open_direc == []:
            print("ERROR: Not Possible to Construct Graph from this Node")
            return None, [x for x in angle_dict if angle_dict[x] is not []]
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


num_junctions = 3
num_roads = 3

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
