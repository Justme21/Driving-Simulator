import linear_controller_profiles as lcp
import math

class DrivingController():

    def __init__(self,controller="standard",speed_limit=22.5,speed_limit_buffer=None,ego=None,other=None,timestep=.1,delay=0,**kwargs):
        if speed_limit_buffer is None:
            speed_limit_buffer = .1*speed_limit

        #Parameters used to regerenerate/reinitialise the controller later
        self.controller_tag = controller
        self.initialisation_params = {'ego':ego,'other':other,'speed_limit':speed_limit,'speed_limit_buffer':speed_limit_buffer,'timestep':timestep}
        self.initialisation_params.update(kwargs)

        self.controller = self.getController(controller,**self.initialisation_params)
        self.ego = None
        self.other = None
        self.setup(ego=ego,other=other)

        self.log = []
        self.speed_limit = speed_limit+speed_limit_buffer

        self.state = []


    def getController(self,controller,**kwargs):
        controller_list = {"standard":lcp.StandardDrivingController,"follow":lcp.FollowController,"generator":lcp.DataGeneratorController,\
                "fast":lcp.GoFastController,"slow":lcp.GoSlowController,"overtake":lcp.OvertakeController,"random":lcp.RandomController}

        return controller_list[controller](**kwargs)


    def setup(self,ego=None,other=None,**kwargs):
        if ego is not None:
            self.ego = ego
            self.initialisation_params['ego'] = ego
        if other is not None:
            self.other = other
            self.initialisation_params['other'] = other

        if ego is not None:
            if isinstance(self.controller,lcp.FollowController):
                self.controller.setLeaderAndFollower(leader=other,follower=ego,**kwargs)
            else:
                self.controller.setup(ego=ego,other=other)


    def getAccelRange(self,state):
        accel_range = self.controller.getAccelRange(state)
        angle_range = self.controller.getAngleRange(state)

        return accel_range,angle_range


    def extractFeatures(self,in_state):
        state = dict(in_state)

        if self.ego is not None:
            ego_vel = self.ego.v
            ego_accel = self.ego.accel
        else:
            ego_vel = 0
            ego_accel = 0

        if self.other is not None:
            other_v = self.other.v
            other_com = tuple([self.other.x_com,self.other.y_com])
            other_accel = self.other.accel
        else:
            other_v = 0
            other_com = tuple([0,0])
            other_accel = 0

        state["velocity"] = ego_vel
        state["other_v"] = other_v
        state["other_position"] = other_com
        state["acceleration"] = ego_accel
        state["other_accel"] = other_accel

        return state


    def inferFeatures(self,state):
        ego_com = state["position"]
        other_com = state["other_position"]

        state["del_d"] = distance(ego_com,other_com)
        state["del_v"] = state["other_v"] - state["velocity"]

        return state


    def defineState(self,in_state):
        state = self.extractFeatures(in_state)
        full_state = self.inferFeatures(state)

        #Note: This is here purely for debugging purposes
        self.state = full_state
        return full_state


    def selectAction(self,in_state,accel_range,angle_range):
        full_state = self.defineState(in_state)
        accel,angle_accel = self.controller.selectAction(full_state,accel_range,angle_range)

        #print("Full State: {}\nAccel:{}\n\n".format(full_state,accel))

        #HACKY: This needs to be removed. Inserted to facilitate first year review results
        next_vel = self.ego.state["velocity"] + self.ego.timestep*accel
        if accel_range == [None,None]:
            accel_range = list(accel_range)
            accel_range = self.controller.accel_range
        if next_vel<0:
            accel = max(accel_range[0],-self.ego.state["velocity"]/self.ego.timestep)
        elif next_vel>self.speed_limit:
            accel = min(accel_range[1],max(accel_range[0],(self.speed_limit-self.ego.state["velocity"])/self.ego.timestep))

        self.log.append([full_state,accel])

        return accel,angle_accel


    def reset(self):
        self.controller = self.getController(self.controller_tag,**self.initialisation_params)


    def getLog(self):
        return self.log


    def clearLog(self):
        self.log = []


    def testAction(self,accel,angle_accel,vel=None,heading=None):
        x_dot,y_dot,v_dot,head_dot = self.ego.simulateDynamics(accel,angel_accel,vel,heading)
        return x_dot,y_dot,v_dot,head_dot


class DelayedDrivingController(DrivingController):

    def __init__(self,delay=0,timestep=.1,**kwargs):
        kwargs.update({'timestep':timestep})
        super().__init__(**kwargs)

        self.action_list = [(0,0) for _ in range(int(delay/timestep))]

    def selectAction(self,*args):
        accel,angle_accel = super().selectAction(*args)
        self.action_list.append((accel,angle_accel))
        print("ACTION_LIST: {}".format(self.action_list))
        (accel,angle_accel) = self.action_list.pop(0)

        return accel,angle_accel


class ProactiveDrivingController(DelayedDrivingController):

    def __init__(self,anticipation_time=0,awareness_coef=.1,non_ego_trajectory=None,timestep=.1,**kwargs):
        kwargs.update({'timestep':timestep})

        super().__init__(self,**kwargs)
        self.timestep = timestep

        if non_ego_trajectory is not None:
            self.other_trajectory = non_ego_trajectory
            self.anticipation_time = anticipation_time
            self.traj_index = 0
            self.awareness =  awareness_coef
        else:
            self.other_trajectory = None
            self.anticipation_time = 0
            self.traj_index = None
            self.awareness = None


    def selectAction(self,in_state,accel_range,angle_range):
        state = super().defineState(in_state)

        if self.other_trajectory is not None and state["other_position"] != self.other_trajectory[self.traj_index][0]:
            print("Error: Controller mismatch\nProactive Controller has NE at: {} and Sub Controller has at: {}".format(self.other_trajectory[self.traj_index][0],state["other_position"]))
            exit(-1)

        tot_accel = [0,0]
        determinant = 0
        for i in range(0,self.anticipation_time+1):
            action = self.controller.selectAction(state,accel_range,angle_range)
            tot_accel = [x+(self.awareness**i)*y for x,y in zip(tot_accel,action)]
            determinant += (self.awareness**i)

            if self.other_trajectory is None or self.traj_index+i+1>=len(self.other_trajectory): break
            else:
                traj_point = self.other_trajectory[self.traj_index+i+1]
                state["other_v"] = traj_point[1]
                state["other_position"] = traj_point[0]
                self["other_accel"] = traj_point[2]

                x_dot,y_dot,v_dot,head_dot = super().testAction(action[0],action[1],state["velocity"],state["heading"])
                state["position"][0] += x_dot*timestep
                state["position"][1] += y_dot*timestep
                state["velocity"] += v_dot*timestep
                state["heading"] += head_dot*timestep
                state = super().inferFeatures(state)

        tot_accel = [x/determinant for x in tot_accel]
        self.traj_index += 1
        return tot_accel[0],tot_accel[1]


    def reset(self):
        super().reset()
        if self.traj_index is not None:
            self.traj_index = 0


def distance(pt1,pt2):
    return math.sqrt((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2)
