# -*- coding: utf-8 -*-
from vehicle_classes import Car

class InteractionConsciousCar(Car):

    def __init__(self,interactive_obstacles=[],interaction_cost=None,**kwargs):
        super().__init__(**kwargs)

        self.interactive_obstacles = interactive_obstacles
        if interaction_cost is None:
            interaction_cost = defaultCost
        self.interactionCost = interaction_cost


    def computeInteractionCost(self):
        ic = 0
        for in_obs in self.interactive_obstacles:
            ic += self.singleInteractionCost(in_obs)

        return ic


    def singleInteractionCost(self,other):
        return self.interactionCost(self.state,other.state)


    def copy(self,dup=None):
        if dup is None or not isinstance(dup,InteractionConsciousCar):
            dup = InteractionConsciousCar(controller=None,label="IDUMMY{}".format(self.label),is_ego=self.is_ego,is_demo=self.in_demo,debug=self.debug,is_interactive=False,timestep=self.timestep)

        dup.interactive_obstacles = self.interactive_obstacles
        dup.interactionCost = self.interactionCost
        dup = super().copy(dup)

        return dup


def defaultCost(*args):
    return 0
