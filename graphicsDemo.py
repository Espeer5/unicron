import pygame
import graphics

pygame.init()

size = (400, 500)
screen = pygame.display.set_mode(size)

done = False

while not done:
    screen.fill((255,0,0))
    pygame.display.flip()


