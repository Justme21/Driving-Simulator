import math
import simulator
import trajectory_builder

def computeDistance(pt1,pt2):
    return math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]+pt2[1])**2)

def moveAhead(num_steps,step_size,cur_posit,waypoints):
    posit = 0
    dist = num_steps*step_size
    waypoints = [cur_posit] + waypoints
    while posit<len(waypoints) and dist>computeDistance(waypoints[posit],waypoints[posit+1]):
        dist -= computeDistance(waypoints[posit],waypoints[posit+1])
        posit += 1

    frac = dist/computeDistance(waypoints[posit],waypoints[posit+1])
    cur_posit = [frac*waypoints[posit][i]+(1-frac)*waypoints[posit+1][i] for i in range(2)]

    return cur_posit


def compareTrajectories(sim,ego,other_guy,N):
    ego_traj_len = 0
    cur_pt = ego.waypoints[0]
    for i in range(1,len(ego.waypoints)):
        ego_traj_len += computeDistance(cur_pt,ego.waypoints[i])
        cur_pt = ego.waypoints[i]
    ego_stepsize = (1.0*ego_traj_len)/N
    t_size = stepsize/ego.v
    other_stepsize = t_size*other.v

    risk = 0
    ego_posit = [ego.x_com,ego.y_com]
    other_posit = [other.x_com,other.y_com]
    for i in range(N):
        ego_posit = moveAhead(1,ego_stepsize,ego_posit,ego.waypoints)
        computeRisk(ego_posit,other_guy.posit



run_graphics = True
draw_traj = True
debug = False

sigma = .01

num_cars = 2
num_junctions = 5
num_roads = 4

init_speeds = [0,0]
starts = [[(3,1),1],[(1,2),0]]
dests = [[(1,4),1]]

junc_pairs = [(0,1),(1,2),(3,1),(1,4)]
road_lengths = [30,30,30,30]
road_angles = [0,0,90,90]

accel_cats = [(-5,-2.5),(-2.5,0),(0,2.5),(2.5,5)]
angle_cats = [(-5,-2.5),(-2.5,0),(0,2.5),(2.5,5)]

num_sample_points = 100

cars = simulator.initialiseControlledCars(num_cars,[],accel_cats,angle_cats,debug)

sim = simulator.Simulator(run_graphics=run_graphics,draw_traj=draw_traj)
sim.loadCars(cars)
sim.initialiseSimulator(num_junctions,num_roads,road_angles,road_lengths,junc_pairs,init_speeds,starts,dests)

compareTrajectories(sim,car[0],car[1],num_sample_points)
