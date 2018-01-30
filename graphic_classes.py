import pygame
import road_classes
import vehicle_classes

screen_width,screen_height = 1024,640
UNIT = 10

BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
DARK_BLUE = (0,0,128)
WHITE = (255,255,255)

INVIS = (255,0,255)

class GraphicWrapper(pygame.sprite.Sprite):
    def __init__(self,obj,surface = None,overwrite=True):
        super().__init__()
        self.obj = obj
        self.image = None
        self.overwrite = overwrite
        if isinstance(obj,vehicle_classes.Car):
            self.default_color = BLUE
        else:
            self.default_color = BLACK
        self.backup_color = GREEN
        if isinstance(obj,road_classes.Lane):
            if not obj.is_top_up:
                self.backup_color = RED
        if surface is None:
            self.surface = pygame.Surface((screen_width,screen_height))
            self.surface.fill(INVIS)
            self.surface.set_colorkey(INVIS)
        else:
            self.surface = surface
      
        if isinstance(obj,road_classes.Road):
            self.extras = [GraphicWrapper(obj.top_up_lane,self.surface,False),\
                           GraphicWrapper(obj.bottom_down_lane,self.surface,False)]
        else:
            self.extras = []


    def draw(self,is_default=True,extra_default=True):
        if self.overwrite:
            self.surface.fill(INVIS)
        crnr = self.obj.four_corners
        corner_list = [crnr["front_left"],crnr["front_right"],crnr["back_right"],\
                       crnr["back_left"]]

        for i,entry in enumerate(corner_list):
            corner_list[i] = [UNIT*k for k in entry]
        if is_default: 
            color = self.default_color
        else:
            color = self.backup_color
        pygame.draw.polygon(self.surface,color,corner_list,0)
        for entry in self.extras:
            entry.draw(extra_default)
        self.rect = self.surface.get_rect()
        self.image = self.surface
