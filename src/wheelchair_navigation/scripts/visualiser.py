# import pygame module in this program
import pygame
from pygame.locals import *
import csv

pygame.init()

white = (255, 255, 255)
red   =     (255, 0, 0)
green =     (0, 255, 0)

# assigning values to X and Y variable
X = 128
Y = 128

# create the display surface object
# of specific dimension..e(X, Y).
master_surface = pygame.display.set_mode((X, Y), HWSURFACE|DOUBLEBUF|RESIZABLE)
display_surface = master_surface.copy()
# set the pygame window name
pygame.display.set_caption('Image')

# create a surface object, image is drawn on it.
image = pygame.image.load(r'/home/raghav/OntarioTech/AutonomousWheelchair/src/wheelchair_navigation/src/sample_maps/map3.pgm')
# scale the image to mathc the resolution of the window
image = pygame.transform.scale(image, (X, Y))

with open('/home/raghav/OntarioTech/AutonomousWheelchair/src/wheelchair_navigation/src/sample_maps/tree_nodes.csv', 'r') as csv_file:
    reader = csv.reader(csv_file, delimiter = ',')
    while True:
        try:
            node_info = next(reader)
            if node_info[0] == '----------':
                pass
            else:
                pass

        except:
            break;


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == VIDEORESIZE:
            master_surface = pygame.display.set_mode((event.w, event.h), RESIZABLE)
    
    display_surface.blit(image, (0, 0)) #load the image on as background
    
    
    
    master_surface.blit(pygame.transform.scale(display_surface, master_surface.get_rect().size), (0, 0))
    pygame.display.update()