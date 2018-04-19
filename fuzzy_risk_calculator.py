import math
import road_classes
import simulator
import time
import trajectory_builder


def normalDistribution(x,avg,std_dev):
    return (1/math.sqrt(2*math.pi*(std_dev**2)))*math.exp(-((x-avg)**2)/(2*std_dev**2))


def computeDistance(pt1,pt2):
    return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]+pt2[1])**2)


def distanceTravelled(u,a,t):
    return u*t + .5*a*(t**2)


def moveAhead(dist,cur_posit,waypoints):
    posit = 0
    dist_orig = dist
    posit_orig = cur_posit
    waypoints = [cur_posit] + waypoints
    while posit+1<len(waypoints) and dist>computeDistance(waypoints[posit],waypoints[posit+1]):
        dist -= computeDistance(waypoints[posit],waypoints[posit+1])
        posit += 1

    if posit+1<len(waypoints):
        frac = dist/computeDistance(waypoints[posit],waypoints[posit+1])
        cur_posit = [(1-frac)*waypoints[posit][i]+frac*waypoints[posit+1][i] for i in range(2)]
    else:
        #This handles the case when the other vehicle drives fast enough to exceed it's trajectory while the other car is still moving
        cur_posit = waypoints[-1]

    return cur_posit


def getPossTrajectories(car):
    traj_dict = {}
    junc = None
    on = None
    for entry in car.on:
        if isinstance(entry,road_classes.Lane):
            on = entry
            junc = entry.to_junction
            break
    dest_weight_list = []
    wrapper = None
    for lane in junc.out_lanes:
        wrapper = []
        wrapper.append(abs(lane.direction-car.heading))
        wrapper.append(trajectory_builder.buildTrajectory(on,lane.to_junction)[1]) #Only keep the trajectory waypoints
        traj_dict[lane] = wrapper
    scale = sum([x[0] for x in traj_dict.values()])
    for lane in junc.out_lanes: traj_dict[lane][0]/=scale
    return traj_dict


def evaluateMetrics(ego_posit,ego_vel,other,time,wayp,accel_range,uncertainty,num_vel_sam):
    rel_vel = 0
    rel_dist = 0
    vel_sum = 0
    prob_sum = 0

    accel,prob = None,None
    other_dist,other_posit = None,None

    accel_avg = (accel_range[0]+accel_range[1])/2
    accel_sigma = 1

    for i in range(num_vel_sam):
        frac = i/num_vel_sam
        accel  = (1-frac)*accel_range[0] + frac*accel_range[1]
        prob = normalDistribution(accel,accel_avg,accel_sigma)
        vel_sum += (other.v+(accel*time))*prob

        other_dist = other.v*time + .5*accel*(time**2)
        other_posit = moveAhead(other_dist,[other.x_com,other.y_com],wayp)
        rel_dist += (computeDistance(ego_posit,other_posit)-uncertainty)*prob
        prob_sum += prob

    vel_sum/=prob_sum
    rel_dist/=prob_sum
    rel_vel = vel_sum - ego_vel
    return rel_dist,rel_vel


def computeMemVal(line_seg,is_low,val):
    if None in line_seg:
        return 1
    else:
        if is_low:
            return (val-line_seg[0])/(line_seg[1]-line_seg[0])
        else:
            return (line_seg[1]-val)/(line_seg[1]-line_seg[0])


def computeMembership(low_threshold,high_threshold,val):
    lower_mem = computeMemVal(low_threshold,True,val)
    upper_mem = computeMemVal(high_threshold,False,val)
    return max(0,min(1,lower_mem,upper_mem))


def fuzzify(dist,vel):
    #DISTANCE: Very Far - Far - Just Right - Close - Very Close
    #VELOCITY: Very Neg - Neg - 0 - Pos - Very Pos
    distance_member = [None for _ in range(5)]
    velocity_member = [None for _ in range(5)]

    print("DIST IS {}".format(dist))

    dist_mem_rules = [((None,None),(3,2)),((2.5,1.5),(1.5,1)),((1.5,1.25),(.75,.5)),((1,.75),(.5,.25)),((.5,0),(None,None))]
    velocity_mem_rules = [((None,None),(-5,-2.5)),((-3,-1.5),(-1.5,0)),((-1,-.5),(.5,1)),((0,1.5),(1.5,3)),((3,5),(None,None))]

    for i,entry in enumerate(dist_mem_rules):
        distance_member[i] = computeMembership(entry[0],entry[1],dist)

    for i,entry in enumerate(velocity_mem_rules):
        velocity_member[i] = computeMembership(entry[0],entry[1],vel)

    return distance_member,velocity_member


def evaluateRules(dist_member,vel_member):
    #DISTANCE: 0:Very Far, 1:Far, 2:Just Right, 3:Close, 4:Very Close
    #VELOCITY: 0:Very Neg, 1:Neg, 2:Zero, 3:Pos, 4:Very Pos
    #RISK: 0:No Risk, 1:Some Risk, 2:Risky, 3:Too Much Risk
    #Hard-Coding Rules
    risk_mem = [0,0,0,0]
    r_temp = min(dist_member[0],max(vel_member)) #R1 in grid
    r_temp = max(r_temp,min(dist_member[1],max(vel_member[2],vel_member[3],vel_member[4])))
    r_temp = max(r_temp,min(dist_member[2],max(vel_member[2],vel_member[3],vel_member[4])))
    risk_mem[0] = r_temp

    r_temp = min(dist_member[1],max(vel_member[0],vel_member[1]))
    r_temp = max(r_temp,min(dist_member[2],vel_member[1]))
    r_temp = max(r_temp,min(dist_member[3],max(vel_member[3],vel_member[4])))
    r_temp = max(r_temp,min(dist_member[4],vel_member[4]))
    risk_mem[1] = r_temp

    r_temp = min(dist_member[2],vel_member[0])
    r_temp = max(r_temp,min(dist_member[3],max(vel_member[1],vel_member[2])))
    r_temp = max(r_temp,min(dist_member[4],max(vel_member[2],vel_member[3])))
    risk_mem[2] = r_temp

    r_temp = min(dist_member[3],vel_member[0])
    r_temp = max(r_temp,min(dist_member[4],max(vel_member[0],vel_member[1])))
    risk_mem[3] = r_temp

    return risk_mem


def defuzzify(risk_membership):
    num_vals = 100
    risk_mem_rules = [((None,None),(.05,.2)),((.1,.2),(.3,.5)),((.25,.4),(.7,.8)),((.6,.8),(None,None))]
    risk = 0
    for i in range(num_vals):
        risk_list = [min(risk_membership[j],computeMembership(risk_mem_rules[j][0],risk_mem_rules[j][1],(1/num_vals)*i)) for j in range(len(risk_membership))]
        #print("CALCULATED RISK {}\t{}".format(i/num_vals,risk_list))
        risk += (1/num_vals)*i*max(risk_list)
    return risk


def fuzzy(dist,vel):
    #DISTANCE: 0:Very Far, 1:Far, 2:Just Right, 3:Close, 4:Very Close
    #VELOCITY: 0:Very Neg, 1:Neg, 2:Zero, 3:Pos, 4:Very Pos
    #RISK: 0:No Risk, 1:Some Risk, 2:Risky, 3:Too Much Risk
    dist_member,vel_member = fuzzify(dist,vel)
    risk_mem = evaluateRules(dist_member,vel_member)
    print("DISTANCE MEMBERSHIP: {}".format(dist_member))
    print("VELOCITY MEMBERSHIP: {}".format(vel_member))
    print("RISK MEMBERSHIP: {}".format(risk_mem))
    risk = defuzzify(risk_mem)
    return risk


def trajectoryRisk(ego,other,uncertainty,other_wayp,accel_range,num_traj_sam,num_vel_sam):
    ego_traj_len = 0
    cur_pt = ego.waypoints[0]
    print("LEN: {}".format(ego.waypoints))
    for i in range(1,len(ego.waypoints)):
        print("CUR: {}\t{}".format(cur_pt,ego.waypoints[i]))
        ego_traj_len += computeDistance(cur_pt,ego.waypoints[i])
        cur_pt = ego.waypoints[i]
    exit(-1)
    print("TRAJ_LEN: {}".format(ego_traj_len))
    ego_stepsize = (1.0*ego_traj_len)/num_traj_sam
    t_size = ego_stepsize/ego.v

    print("T_SIZE: {}\tSTEPSIZE: {}".format(t_size,ego_stepsize))
    exit(-1)

    ego_posit = [ego.x_com,ego.y_com]
    other_posit = None
    risk,t_risk = 0,0
    for i in range(num_traj_sam):
        ego_posit = moveAhead(ego.v*t_size,ego_posit,ego.waypoints)
        print("EGO_POSIT: {}".format(ego_posit))
        dist,vel = evaluateMetrics(ego_posit,ego.v,other,i*t_size,other_wayp,accel_range,uncertainty,num_vel_sam)
        t_risk = fuzzy(dist,vel)
        #print("T_RISK HERE IS: {}".format(t_risk))
        risk += t_risk
    risk/=num_traj_sam
    return risk


def compareTrajectories(sim,ego,other_guy,uncertainty,accel_range,N,M):
    risk = 0
    t_risk = 0
    ego_posit = [ego.x_com,ego.y_com]
    poss_other_traj = getPossTrajectories(other_guy)
    print("POSS_TRAJ: {}".format(poss_other_traj))
    for entry in poss_other_traj:
        t_risk = poss_other_traj[entry][0]*trajectoryRisk(ego,other_guy,uncertainty,poss_other_traj[entry][1],accel_range,N,M)
        #print("T_RISK IS: {} ({})".format(t_risk,poss_other_traj[entry][0]))
        risk += t_risk
    print("RISK IS: {}".format(risk))


run_graphics = True
draw_traj = True
debug = False

sigma = .1

#Information about acceleration values at https://hypertextbook.com/facts/2001/MeredithBarricella.shtml
a_bounds = [0,2.5]

num_cars = 2
num_junctions = 5
num_roads = 4

init_speeds = [5,0]
starts = [[(3,1),1],[(1,2),0]]
dests = [[(0,1),0]]

junc_pairs = [(0,1),(1,2),(3,1),(1,4)]
road_lengths = [30,30,30,30]
road_angles = [0,0,90,90]

accel_cats = [(-5,-2.5),(-2.5,0),(0,2.5),(2.5,5)]
angle_cats = [(-5,-2.5),(-2.5,0),(0,2.5),(2.5,5)]

num_sample_points = 5
num_vel_samples = 10

cars = simulator.initialiseControlledCars(num_cars,[],accel_cats,angle_cats,debug)

sim = simulator.Simulator(run_graphics=run_graphics,draw_traj=draw_traj)
sim.loadCars(cars)
sim.initialiseSimulator(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,init_speeds,starts,dests)
sim.drawSimulation()

t0 = time.time()
compareTrajectories(sim,cars[0],cars[1],sigma,a_bounds,num_sample_points,num_vel_samples)
t1 = time.time()
print("RUNTIME IS: {}".format(t1-t0))
