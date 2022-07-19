# import pygame module in this program
import pygame
from pygame.locals import *
import csv

pygame.init()

white = (255, 255, 255)
red   =     (255, 0, 0)
green = (100, 255, 100)

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
image = pygame.image.load(r'/home/raghav/OntarioTech/AutonomousWheelchair/src/wheelchair_navigation/src/sample_maps/map.pgm')
# scale the image to mathc the resolution of the window
image = pygame.transform.scale(image, (X, Y))
display_surface.blit(image, (0, 0)) #load the image on as background

# coordinates of root and child are string with , in between
def showEdge(root_str, child_str):
    # convert to list from string
    root_coords_str = root_str.split(',')
    child_coords_str = child_str.split(',')
    
    # convert to integer tuple to plot
    root_coords = tuple(map(int, root_coords_str))
    child_coords = tuple(map(int, child_coords_str))

    # plot the edge
    pygame.draw.line(display_surface, red, root_coords, child_coords)

with open('/home/raghav/OntarioTech/AutonomousWheelchair/src/wheelchair_navigation/src/sample_maps/tree_nodes.csv', 'r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=':')

    # loop through the tree
    for nodes_list in csv_reader:
        root_node = nodes_list[0]
        child_nodes = nodes_list[1:]
        
        # loop through connections and show edge
        for child_node in child_nodes:
            showEdge(root_node, child_node)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == VIDEORESIZE:
            master_surface = pygame.display.set_mode((event.w, event.h), RESIZABLE)
    
    master_surface.blit(pygame.transform.scale(display_surface, master_surface.get_rect().size), (0, 0))
    pygame.display.update()