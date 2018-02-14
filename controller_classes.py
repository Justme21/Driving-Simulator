import math
import random
import road_classes
import vehicle_classes

class Controller():
    def __init__(self,action_list,car,map):
        #The lists that will store the per-use values for the features being used
        # and the associated weights
        self.weights = []
        self.features = []

        #The master lists of trained weights grouped by context
        self.trained_in_weights = [] #Weights for ego vehicle features
        self.trained_env_weights = {} #Weights for environment features
        self.trained_veh_weights = [] #Weights for other vehicle features

        self.weights = {"internal": self.trained_in_weights,\
                        "environment": self.trained_env_weights,\
                        "vehicles": self.trained_veh_weights}

        self.total_reward = 0

        #Learning rate and horizon depth
        self.alpha = .01
        self.gamma = .5

        #The list of discrete actions the controller can choose 
        self.action_list = action_list
        self.weights_unset = True
       
        self.car = car

        self.weight_list = []
        self.reward_list = []


    def getInternalFeatures(self):
        feature_list = []
        ego = self.car
        
        feature_list.append(ego.v>50)
        feature_list.append(ego.v>40)
        feature_list.append(ego.v>30)
        feature_list.append(ego.v>20)
        feature_list.append(ego.v>10)
        feature_list.append(ego.on_road)
        feature_list.append(ego.crashed)

        return feature_list

    def getEnvironmentFeatures(self,env):
        feature_list = []
        ego = self.car

        if isinstance(env,road_classes.Lane):
            #Binary feature: if car is going the direction intended by the road
            feature_list.append(int(ego.heading/180)==int(env.direction/180))
            if int(ego.heading/180) == int(env.direction/180):
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["front_left"][0])<env.width/3)
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["front_left"][0])<env.width/2)
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["front_left"][0])>2*env.width/3)
                feature_list.append(math.fabs(env.four_corners["front_left"][1]-\
                        ego.four_corners["front_left"][1])<ego.length)
                feature_list.append(math.fabs(env.four_corners["front_left"][1]-\
                        ego.four_corners["front_left"][1])>env.length/5)
            else:
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["back_right"][0])<env.width/3)
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["back_right"][0])<env.width/2)
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["back_right"][0])>2*env.width/3)
                feature_list.append(math.fabs(ego.four_corners["front_left"][0]-\
                        env.four_corners["back_right"][0])<env.width/3)
                feature_list.append(math.fabs(env.four_corners["front_left"][1]-\
                        ego.four_corners["back_right"][1])<ego.length)
                feature_list.append(math.fabs(env.four_corners["front_left"][1]-\
                        ego.four_corners["back_right"][1])>ego.length/5)


        elif isinstance(env,road_classes.Junction):



    def getFeatures(self,action):
        feature_list = []
        feature_list.append(self.getInternalFeatures())
        for env in self.on:
            feature_list.append(self.getEnvironmentFeatures(env))
            for car in [x in env.on if x is not self.car]:
                feature_list.append(self.getVehicleFeatures())
        
        if self.weights_unset: self.weights_unset = False
