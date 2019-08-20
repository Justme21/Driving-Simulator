import pygame
import road_classes
import vehicle_classes

BLACK = (0,0,0)
RED = (255,0,0)
GREEN = (0,255,0)
BLUE = (0,0,255)
DARK_BLUE = (0,0,128)

class GraphicWrapper(pygame.sprite.Sprite):
    def __init__(self,obj,unit=1):
        super().__init__()

        self.obj = obj
        if isinstance(obj,vehicle_classes.Car):
            if obj.is_ego:
                self.image = pygame.image.load("/Users/JackGeary 1/Documents/College/University of Edinburgh/PhD Stuff/Programs/driving_simulator/car-blue.png")
                #self.default_color = BLUE
            else:
                self.image = pygame.image.load("/Users/JackGeary 1/Documents/College/University of Edinburgh/PhD Stuff/Programs/driving_simulator/car-red.png")
                #self.default_color = RED
            self.image = pygame.transform.scale(self.image,(int(obj.length*unit),int(obj.width*unit)))
            centre_point = (self.obj.x_com,self.obj.y_com)
        else:
            self.image = pygame.Surface((int(obj.length*unit),int(obj.width*unit))) #this is widthxlength. By default these shapes begin facing 0 degrees, which is to screen right. So it is l wide and w high ("long")
            self.image.fill(BLACK)
            self.image = pygame.transform.rotate(self.image,obj.direction)
            corners = list(self.obj.four_corners.values())
            centre_point = (sum([x[0] for x in corners])/len(corners),sum([x[1] for x in corners])/len(corners))

        self.rect = self.image.get_rect() #the bounding box for the image
        self.rect.center = (int(centre_point[0]*unit),int(centre_point[1]*unit)) #position the box containing the image (and therefore the image) to be where the symbolic object is

        if isinstance(obj,road_classes.Road):
            #Generate the road lanes
            self.extras = [GraphicWrapper(obj.top_up_lane,unit=unit),\
                       GraphicWrapper(obj.bottom_down_lane,unit=unit)]
        else:
            self.extras = []


    def draw(self,screen,unit):
        """Called on each iteration of the graphical simulation. Draws the graphical depiction of the object"""
        if isinstance(self.obj,vehicle_classes.Car):
            #Rotate the image each time instead of keeping track of heading
            image = pygame.transform.rotate(self.image,self.obj.heading)
            rect = image.get_rect()
            rect.center = (int(self.obj.x_com*unit),int(self.obj.y_com*unit))

        else:
            #no need to redconstruct roads and junctions as they are stationary and unchanging.
            image = self.image
            rect = self.rect

        #Draw the image of the object onto the screen
        screen.blit(image,rect)

        if isinstance(self.obj,road_classes.Road):
            #Draw the lanes on the road. Right now they are also jut black rectangles, but hopefully we'll add details later 
            for x in self.extras: x.draw(screen,unit)
