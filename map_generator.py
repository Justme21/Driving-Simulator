import math
import random
import simulator

#random.seed(493072)

REVERSE_DICT = {"top":"bottom","bottom":"top","left":"right","right":"left"}

class Graph():
    def __init__(self,debug):
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

        #Number of times it has been attempted to fix an incomplete graph
        self.try_count = 0

        self.debug = debug

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

    def removeLink(self,link):
        link.removeFromNodes()
        self.link_list.remove(link)
        self.num_links -= 1
        del[link]

    def removeNode(self,node):
        if self.debug: print("REMOVING {}: {}".format(node.label,node.coordinates))
        self.num_nodes -= 1
        self.coords.remove(node.coordinates)
        posit = self.node_list.index(node)
        for entry in self.node_list:
            if entry.label > node.label:
                entry.label -= 1
        self.node_list.remove(node)
        if self.debug: print("COORD CHANGE: {}".format(self.coords))
        del[node]


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
        angle = None
        #The number of links that will be attempted to be created this round
        if min(link_limit-self.num_links,len(open_direc))>0:
            num_links = random.randint(1,min(link_limit-self.num_links,len(open_direc)))
        else:
            #In the case there are no more links to add randint passes an error so handle
            # separately. Also since num_links = 0 then you never enter the for loop
            # where false is initialised, so intialise here to allow exit from the loop
            num_links = 0
            fail = False
        if self.debug:
            print("LABEL: {}\tAVAIL: {}".format(node.label,open_direc))
            print("CONSTRAINTS: {}".format(direc_constr))
            print("TRYING TO PRINT {} LINKS".format(num_links))
        #At the back end of the recursion the node might not exist
        if node not in self.node_list:
            node = random.choice(self.node_list)
            direc_constr = []
        #Coordinates of the current node
        node_coords = self.coords[self.node_list.index(node)]
        new_links = []
        new_direcs = []
        for i in range(num_links):
            #getAngle chooses an angle from angle_dict that corresponds to an available side
            # of the current node. If getAngle fails to find any such angle it leaves angle
            # as none and sets direc to be the set of directions for which there are angles
            # yet unassigned
            angle,direc = getAngle(open_direc,direc_constr,angle_dict,self.debug)
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
                        if entry.directions[avail] is None:
                            open_nodes.append(entry)
                            #Don't need to iterate over all angles if node works for any one
                            break
                #Keep checking while there are still nodes to check and angle is not assigned
                while angle is None and open_nodes != []:
                    #These are the same steps that occur at the start of the program
                    if node in open_nodes: open_nodes.remove(node)
                    node = random.choice(open_nodes)
                    if self.debug:
                        print("Moved to Node {} ({})".format(node.label,node.coordinates))
                    new_direcs = []
                    node_coords = self.coords[self.node_list.index(node)]
                    open_direc = [x for x in node.directions if node.directions[x] is None]
                    angle,direc = getAngle(open_direc,[],angle_dict,self.debug)
                if open_nodes == []:
                    print("ERROR: No Suitable Nodes Available. Terminating")
                    exit(-1)
            #By this point we have chosen a direction, we add it to the list of directions
            # being used in this round
            new_direcs.append(direc)
            #Create the angle being inserted this round
            link = Link(angle)
            fail = True #Fail is true if we fail to insert the link by the end of the round
            #Copy the coordinates of the current node that will be modified to be the 
            # coordinates for the node the link will connect to

            #Index is the index of the non-zero entry in the link.change list
            if math.fabs(link.change[0]) == 1:index = 0
            else: index = 1

            #We only consider nodes whose unchanged coord matches the current coord
            # in the direction of the change
            potentials = []
            if self.debug: print("Beginning this fix")
            pot_coords = None
            for pot_to_node in self.node_list:
                pot_coords = self.coords[self.node_list.index(pot_to_node)]
                change = [pot_coords[i]-node_coords[i] for i in range(2)]
                for i in range(len(change)):
                    if change[i]!=0: change[i]/=math.fabs(change[i])
                if change[1-index] == 0 and (change[index]/link.change[index])>0:
                    if change == [1,0]: pot_direc = "right"
                    elif change == [-1,0]: pot_direc = "left"
                    elif change == [0,1]: pot_direc = "top"
                    elif change == [0,-1]: pot_direc = "bottom"
                    
                    if pot_to_node.directions[pot_direc] is None:
                        potentials.append(pot_coords)

            #Looking for the closest node to our current coordinate
            node_exists = False #indicates if there is a preexisting node to link to
            min_dist = -1 #initialise the minimal distance between nodes
            for coord in potentials:
                if min_dist == -1 or math.fabs(coord[index]-node_coords[index])<min_dist:
                    min_dist = math.fabs(coord[index] - node_coords[index])
                    min_coords = coord
                    node_exists = True

            #There already exists a node in the graph with the same coordinates as the 
            # node the link will connect to
            if node_exists:
                #set the target node to the the corresponding entry in the list of nodes
                new_node = self.node_list[self.coords.index(min_coords)]
                if self.debug:
                    print("Found a node to connect to: {} ({})".format(new_node.label,new_node.coordinates))
                #Identified target node is suitable for connection
                # In this case we add link to graph, remove angle from available angles
                # and remove direction from list of available directions on the node
                if new_node.directions[REVERSE_DICT[direc]] is None:
                    self.addLink(link,node,new_node,direc)
                    if self.debug:
                        print("Created New Link: {}\t{}\t{}".format(link.from_node.coordinates,link.to_node.coordinates,link.angle))
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
                if self.debug: print("There is no node available to link to")
                new_coords = list(node_coords)
                for i in range(2): new_coords[i] += link.change[i]
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
                    if self.debug:
                        print("Created New Node {}: {}\t({})".format(new_node.label,direc, new_coords))
                        print("Created New Link: {}\t{}\t{}".format(link.from_node.coordinates,link.to_node.coordinates,link.angle))
                    #We have not failed to create a new link
                    fail = False
                #If the above does not trigger we have faile to make a link
        if fail:
            if self.debug:
                print("Tried these directions: {} and failed".format(new_direcs))
        else:
            if self.debug:
                print("Succeeded in creating a link: {}".format(direc))
        #exit(-1)
        if fail:
            #If fail is true we have not made any new links this round
            if self.debug: print("Still in same node")
            #We remove the directions we have previously tried this round from consideration
            complete = True
            for entry in new_direcs:
                if entry not in direc_constr:
                    direc_constr.append(entry)
                    complete = False
            if complete:
                if self.debug:
                    print("There's something wrong here")
                    print("REMAINING ANGLES: {}".format(angle_dict))
                self.try_count += 1
                expendable = [x for x in self.node_list if len([y for y in x.directions if x.directions[y] is not None])==1]
                if expendable == [] or self.try_count>2*node_limit:
                    if self.debug:
                        if self.try_count>2*node_limit: print("Giving Up")
                        print("Cannot make full graph")
                    #exit(-1)
                else:
                    spare_node = random.choice(expendable)
                    for entry in spare_node.directions:
                        if spare_node.directions[entry] is not None:
                            spare_link = spare_node.directions[entry]
                            if spare_node is spare_link.from_node:
                                angle_dict[entry].append(spare_link.angle)
                            else:
                                angle_dict[REVERSE_DICT[entry]].append(spare_link.angle)
                            self.removeLink(spare_node.directions[entry])
                            break #by definition there is only one filled direction on this node.
                    if self.debug: print("Removing node at ({})".format(spare_node.coordinates))
                    self.removeNode(spare_node)
                    if spare_node is node:
                        node = self.node_list[0]
                    if self.debug: 
                        print("Trying to build graph from node {} ({}) with no constraints".format(node.label,node.coordinates))
                    self.buildGraph(angle_dict,node,node_limit,link_limit,direc_constr=[])
                    if self.debug: 
                        print("Back in {} ({}) after trying to fix what was wrong".format(node.label,node.coordinates))
                #exit(-1)
            else:
                #Recursively try again building on the same node, but with the new constraints
                if self.debug:
                    print("Trying to build graph from node {} ({}) with constraints {}".format(node.label,node.coordinates,direc_constr))
                self.buildGraph(angle_dict,node,node_limit,link_limit,direc_constr)
                if self.debug:
                    print("Back in {} ({}) after trying to put a link on this node with extra constraints".format(node.label,node.coordinates))

        else:
            #We have successfully constructed at least one link. We move onto the new nodes
            # and try make new links from there (if we need to make more links)
            if self.debug: print("Going onto other nodes")
            for link in new_links:
                if self.num_links<link_limit:
                    #If more links to be made make them off the new connections
                    self.buildGraph(angle_dict,link.to_node,node_limit,link_limit)
                    if self.debug:
                        print("Back in {} ({}) after making other nodes still in loop".format(node.label,node.coordinates))
                else:
                    #This is the end condition for the loop that does not exit out of the
                    # program
                    pass


    def printStatus(self):
        """The graph can print it's status for debugging purposes (and also to inspect the
           connections created)"""
        print("NODES: {}\nLINKS: {}".format(self.num_nodes,self.num_links))
        for entry in self.link_list:
            print("OUT: {} {}\tANGLE: {}\tIN: {} {}".format(entry.from_node.label,\
                      entry.from_node.coordinates,entry.angle,entry.to_node.label,\
                      entry.to_node.coordinates))


class Node():
    def __init__(self,i,j,k):
        self.label = k
        self.coordinates = (i,j)
        self.directions = {"top":None,"bottom":None,"left":None,"right":None}


class Link():
    def __init__(self,angle):
        #Link contains the angle as a label as well as references to nodes it links to
        self.angle = angle
        self.length = None
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


    def removeFromNodes(self):
        for node in [self.from_node,self.to_node]:
            for direc in node.directions:
                if node.directions[direc] is self:
                    node.directions[direc] = None
                    break


    def otherNode(self,node):
        if node is self.from_node:
            return self.to_node
        else:
            return self.from_node


def getAngle(open_direc,direc_constr,angle_dict,debug):
    """Given a list of availabl directions, and the generated angles associated with each
       direction, returns an angle and direction that can be used to attempt to create a
       link"""
    angle = None
    angle_list = None
    open_direc = list(open_direc)
    open_direc = [x for x in open_direc if x not in direc_constr]
    if debug:
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
            if debug: print("ERROR: Not Possible to Construct Graph from this Node")
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


def probAngleSetter(num_angles,rnge,debug):
    prob_angle_list = []
    angle = None
    for i in range(num_angles):
        angle = random.randint(rnge[0],rnge[1])
        if debug:
            if angle in range(46,135): angle = 90
            elif angle in range(135,226): angle = 180
            elif angle in range(226,315): angle = 270
            else: angle = 0
        prob_angle_list.append(angle)
    return prob_angle_list


def randomiseAngles(count,num_junctions,ang_list):
    randomiser = None
    for i,entry in enumerate(ang_list):
        randomiser = random.choice([True,False])
        if count<num_junctions and randomiser:
            ang_list[i] = (entry+180)%360
            count += 1
    return ang_list,count


def generateAngles(num_junctions,num_roads,spec_num_up,debug):
    num_up = None
    angles = []
    count = None

    if spec_num_up is None:
        while num_up is None or num_up > 2*num_junctions:
            num_up = random.randint(0,2*num_junctions)
    else:
        num_up = spec_num_up

    up_angles = [x for x in probAngleSetter(num_up,(46,135),debug)]
    left_angles = [x for x in probAngleSetter(num_roads-num_up,(135,226),debug)]

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


def assignEndLinkLengths(links,len_range):
    for link in links:
        if len([x for x in list(link.from_node.directions.values())\
                if x in links])==1:
            link.length = random.randint(len_range[0],len_range[1])
            links.remove(link)
        elif len([x for x in list(link.to_node.directions.values())\
                if x in links])==1:
            link.length = random.randint(len_range[0],len_range[1])
            links.remove(link)
    return links


def dist(coord1,coord2):
    dist = 0
    for i in range(len(coord1)):
        dist+=(coord2[i]-coord1[i])**2
    return math.sqrt(dist)


def setScaling(scale_list,len_range):
    scale = list(set(scale_list))
    scale_list = [x/scale[-1] for x in scale_list]
    scale = [x/scale[-1] for x in scale]
    new_scale_list = []
    pos,lo_bound,up_bound = None,None,None
    for entry in scale_list:
        pos = scale.index(entry)
        lo_bound = min(len_range[0],len_range[1]*scale[pos-1])
        up_bound = scale[pos]*len_range[1]
        new_scale_list.append((lo_bound,up_bound))
    return new_scale_list


def setLinkLengths(link_list,len_range,debug):
    links = list(link_list)
    prev_size = None
    cur_size = len(links)
    while prev_size is None or cur_size<prev_size:
        links = assignEndLinkLengths(links,len_range)
        prev_size = cur_size
        cur_size = len(links)
    #All the links remaining in links do not exclusively attach to a 
    # terminal link. Thus links now only contains links in loops
    linger_index = 0
    repeat = True
    cemented = False
    temp_pos = None
    len_list = None
    loop_list,mod_list,scale_list = None,None,None
    next_links = []
    linger_list = [x for x in links if x.length is None]
    if debug:
        print("The links that have lingered: {}".format([(x.from_node.label,x.to_node.label) for x in linger_list]))
    while len(linger_list)>0 and linger_index >= 0:
        link = linger_list[linger_index]
        cur_node = link.from_node
        loop_list = []
        mod_list = []
        scale_list = []

        while link not in loop_list:
            loop_list.append(link)
            mod_list.append(-1+2*(cur_node == link.from_node)) #If true mod is 1, else is -1
            scale_list.append(dist(link.from_node.coordinates,link.to_node.coordinates))
            if debug:
                print("LINK:{}-{}\tMOD: {}".format(link.from_node.label,link.to_node.label,mod_list[-1]))
            cur_node = link.otherNode(cur_node)
            next_links = [x for x in list(cur_node.directions.values()) if x in linger_list\
                    and x is not link]
            if len(next_links)>1:
                for entry in next_links:
                    if entry.length is not None:
                        next_links.remove(entry)
                        break
            min_diff = 359
            min_link = None
            for entry in next_links:
                diff = entry.angle - (mod_list[-1]==1)*180 - link.angle
                if diff<min_diff:
                    min_link = entry
            link = min_link
        #The loop broke because we found a repeat => loop
        #This ensures only elements in the loop are considered
        temp_pos = loop_list.index(link)
        mod_list = mod_list[temp_pos:] #Change mod_list first since afterwards link will not be in same place
        loop_list = loop_list[temp_pos:]
        scale_list = scale_list[temp_pos:]
        if debug:
            print("This is a loop: {}".format([((x.from_node.label,x.to_node.label),mod_list[i]) for i,x in enumerate(loop_list)]))
        while loop_list[-1].angle%180 == loop_list[-2].angle%180:
            loop_list = [loop_list[-1]] + loop_list[:-1]
            mod_list = [mod_list[-1]] + mod_list[:-1]
            scale_list = [scale_list[-1]] + scale_list[:-1]
        temp_list = [x.length is not None for x in loop_list]
        if True in temp_list:
            temp_pos = temp_list.index(True)
            loop_list = loop_list[temp_pos:]+list(loop_list[:temp_pos])
            mod_list = mod_list[temp_pos:]+list(mod_list[:temp_pos])
            scale_list = scale_list[temp_pos:]+list(scale_list[:temp_pos])
        if debug:
            print("This is Loop_List Now: {}".format([((x.from_node.label,x.to_node.label),mod_list[i]) for i,x in enumerate(loop_list)]))
        scale_list = setScaling(scale_list,len_range)
        cemented = False
        while not cemented:
            len_list = [x.length for x in loop_list]
            horiz_sum = 0
            vert_sum = 0
            ang = 0
            if debug:
                print("\n\nENTER LENGTH ASSIGNMENT TEST")
            for i,entry in enumerate(loop_list[:-2]):
                ang = entry.angle
                if mod_list[i] == -1: ang = (ang+180)%360
                if len_list[i] is None:
                    len_list[i] = random.randint(scale_list[i][0],scale_list[i][1])
                horiz_sum += len_list[i]*math.cos(math.radians(ang))
                vert_sum += len_list[i]*math.sin(math.radians(ang))
            if debug:
                print("X: {}\tY: {}\n".format(horiz_sum,vert_sum))

            [l2,l1] = loop_list[-2:]
            a2 = (l2.angle+(mod_list[-2]==-1)*180)%360
            a1 = (l1.angle+(mod_list[-1]==-1)*180)%360
            a2_rad = math.radians(a2)
            a1_rad = math.radians(a1)

            if l2.length is None:
                len_list[-2] = (horiz_sum*math.sin(a1_rad)-vert_sum*math.cos(a1_rad))/\
                         (math.sin(math.radians(a2-a1)))

            if len_list[-1] is None:
                if l1.angle%180 != 0:
                    len_list[-1] = (-vert_sum-len_list[-2]*math.sin(a2_rad))/(math.sin(a1_rad))
                else:
                    len_list[-1] = (-horiz_sum-len_list[-2]*math.cos(a2_rad))/(math.cos(a1_rad))

            if debug:
                test_sum_cos  = 0
                test_sum_sin  = 0
                printout_cos = ""
                printout_sin = ""
                linkprintout = ""
                for i in range(len(loop_list)):
                    printout_cos += "({}*{}*{})+".format(mod_list[i],len_list[i],math.cos(math.radians(loop_list[i].angle)))
                    printout_sin += "({}*{}*{})+".format(mod_list[i],len_list[i],math.sin(math.radians(loop_list[i].angle)))
                    linkprintout += "({}-{})+".format(loop_list[i].from_node.label,loop_list[i].to_node.label)
                    test_sum_cos += mod_list[i]*len_list[i]*math.cos(math.radians(loop_list[i].angle))
                    test_sum_sin += mod_list[i]*len_list[i]*math.sin(math.radians(loop_list[i].angle))

                print("TESTING OUTPUT")
                print("{}\n{}\nTOTAL; SIN: {}\n{}\nTOTAL;COS: {}".format(linkprintout[:-1],printout_sin[:-1],test_sum_sin,printout_cos[:-1],test_sum_cos))

            if True in [x<0 for x in len_list]:
                print("DISREGARD ABOVE")
            else: cemented = True

        for i,entry in enumerate(loop_list):
            entry.length = len_list[i]

        linger_index = -1
        for i,entry in enumerate(linger_list):
            if debug:
                print("{}-{}\t{}".format(entry.from_node.label,entry.to_node.label,entry.length))
            if entry.length is None: linger_index = i
        if debug:
            print("\n")

    if debug:
        print("LENGTH RESULT: {}".format([((x.from_node.label,x.to_node.label),x.length) for x in link_list]))
    return [round(x.length,5) for x in link_list]


def constructSimulation(graph,num_cars,link_len_range,run_graphics,debug):
    num_junctions = len(graph.node_list)
    num_roads = len(graph.link_list)

    road_angles = [x.angle for x in graph.link_list]
    #NOTE: This is a temporary fix. When loops emerge this will not be good enough
    #road_lengths = [random.randint(25,100) for _ in range(num_roads)]
    road_lengths = setLinkLengths(graph.link_list,link_len_range,debug)

    junc_pairs = []
    pre,post = None,None
    for link in graph.link_list:
        pre = link.from_node
        post = link.to_node
        if link.angle>90 and link.angle<=270:
            junc_pairs.append((post.label,pre.label))
            link.angle-=180
        else:
            junc_pairs.append((pre.label,post.label))
        print("{}-{}\t{}\t{}".format(link.from_node.label,link.to_node.label,junc_pairs[-1],link.length))
    
    if not debug:
        simulator.runSimulation(num_junctions,num_roads,num_cars,road_angles,road_lengths,\
                               junc_pairs,run_graphics)

num_junctions = 6
num_roads = 6
num_cars = 1
run_graphics = True

debug_angle = False
debug_graph = False
debug_construction = False

link_len_range = [25,100]

num_up = 4
angles = None

if angles is None or len(angles)!= num_roads or  angleTest(angles,num_junctions,num_roads)!=0:
    angles = generateAngles(num_junctions,num_roads,num_up,debug_angle)

sorted_angles = partitionAngles(angles)
if debug_angle: print("SORTED: {}".format(sorted_angles))
temp = None
for entry in sorted_angles:
    while len(sorted_angles[entry])>num_junctions:
        temp = sorted_angles[entry][0]
        sorted_angles[entry].remove(temp)
        sorted_angles[REVERSE_DICT[entry]].append(temp)

graph = Graph(debug_graph)
graph.buildGraph(sorted_angles,graph.node_list[0],num_junctions,num_roads)
graph.printStatus()
constructSimulation(graph,num_cars,link_len_range,run_graphics,debug_construction)
