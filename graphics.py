import pygame
import MapGraph

class GraphicsEngine:

    def __init__(self, graph):
        self.graph = graph
        pygame.init()
        self.window = (500, 500)
        size(self.window(0), self.window(1))
        self.screen = pygame.display.set_mode(size)
        pygame.display.set_caption("Robonorm Tracker")

    def graphics_show():
        self.screen.fill(WHITE)
        
        #for intersection in self.graph:
        pygame.draw.circle(self.screen, (255, 0, 0), (200, 200), 50);   

        pygame.display.flip()
    


    def graphics_quit():
        pygame.quit()
