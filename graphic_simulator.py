import graphic_classes
import pygame

WHITE = (255,255,255)


class GraphicSimulator():
    def __init__(self,junctions,roads,cars):
        pygame.init()
        
        height = 640
        width = 1024

        self.junc_list = makeSpriteGroup(junctions,width,height)
        self.road_list = makeSpriteGroup(roads,width,height)
        self.car_list = makeSpriteGroup(cars,width,height)

        self.is_quit = False 

        self.initialiseBackground(width,height)

        self.clock = pygame.time.Clock()


    def initialiseBackground(self,width,height):
        self.screen = pygame.display.set_mode([width,height])
        self.background = pygame.Surface(self.screen.get_size())
        self.background.fill(WHITE)


    def update(self):
        for lizt in [self.junc_list,self.road_list,self.car_list]:
            lizt = updateList(lizt)

        self.checkForQuit()

        if not self.is_quit:
            self.screen.blit(self.background,(0,0))
            pygame.display.update()
            for lizt in [self.junc_list,self.road_list,self.car_list]:
                self.drawToScreen(lizt)

            self.clock.tick(10)
            pygame.display.flip()


    def drawToScreen(self,obj_list):
        obj_list.draw(self.screen)


    def checkForQuit(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                self.is_quit = True


def updateList(obj_list):
    for entry in obj_list:
        entry.draw()


def makeSpriteGroup(obj_list,width,height):
    objs = [graphic_classes.GraphicWrapper(x,width,height) for x in obj_list]
    g_obj_list = pygame.sprite.Group()
    for entry in objs:
        entry.draw()
        g_obj_list.add(entry)
    return g_obj_list
