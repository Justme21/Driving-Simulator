import random
import vehicle_classes

class Controller():
    def __init__(self,action_list,car,map):
        #The lists that will store the per-use values for the features being used
        # and the associated weights
        self.weights = []
        self.features = []

        #The master lists of trained weights grouped by context
        self.trained_in_weights = {} #Weights for ego vehicle features
        self.trained_env_weights = {} #Weights for environment features
        self.trained_veh_weights = {} #Weights for other vehicle features

        self.weight_dicts = {"internal": self.trained_in_weights,\
                             "environment": self.trained_env_weights,\
                             "vehicles": self.trained_veh_weights}

        self.total_reward = 0

        #Learning rate and horizon depth
        self.alpha = .01
        self.gamma = .5

        #The list of discrete actions the controller can choose 
        self.action_list = action_list
        for weight_dict in self.weight_dicts:
            for entry in self.action_list:
                weight_dict[enty] = []
        self.weights_unset = True
       
        self.car = car

        self.weight_list = []
        self.reward_list = []


    def getInternalFeatures():
        feature_list = []
        ego = self.car
        
        feature_list.append(ego.v)
        feature_list.append(ego.heading)
        feature_list.append(ego.on_road)
        feature_list.append(ego.crashed)

        if self.weights_unset:
            for entry in self.weight_dicts["internal"]:
                for _ in range(len(feature_list)):
                    self.weight_dicts["internal"][entry].append(random.randint(-10,10)/10)

        return feature_list

    def getEnvironmentFeatures():
        feature_list = []
        ego = self.car

        feature_list.append

        

    def getFeatures(self,action):
        feature_list = []
        feature_list.append(self.getInternalFeatures())
        feature_list.append(self.getEnvironmentFeatures())
        feature_list.append(self.getVehicleFeatures())
        
        if self.weights_unset: self.weights_unset = False
