
import math
from pickletools import TAKEN_FROM_ARGUMENT8U
import pygame
from pygame.locals import *
import sys
from scipy.spatial.distance import pdist
from scipy.spatial.distance import squareform
import time
pygame.init()
pygame.font.init()
my_font = pygame.font.SysFont('Comic Sans MS',27)
import numpy as np
import matplotlib
matplotlib.use('module://pygame_matplotlib.backend_pygame')
import matplotlib.pyplot as plt

NUM_ROBOTS = 4
SLOW_DOWN = False
WINDOW_WIDTH = 1100
WINDOW_HEIGHT = 700


MALICIOUS_AGENTS = [1,3]
K = 2
ROBOT_WINDOW_WIDTH = 700
ROBOT_WINDOW_HEIGHT = 700
TRAJECTORY_SIZE = 20000

colors =  [ list(np.random.choice(range(256), size=3))  for i in range(NUM_ROBOTS) ]

trajectories = [ [None for j in range(TRAJECTORY_SIZE)] for i in range(NUM_ROBOTS) ]
trajectory_index = [ 0 for i in range(NUM_ROBOTS)]

ROBOT_RADIUS = 10
SAFE_REGION = ROBOT_RADIUS+80
K_p = 0.004
ALPHA = 4
BETA = 0.03
K_tracking_control = 0.05


square_template = np.array([[1,-1],[-1,-1],[1,1],[-1,1]])

b_matrix = -K_p*80*square_template

laplacian = np.array([[3,-1,-1,-1],[-1,3,-1,-1],[-1,-1,3,-1],[-1,-1,-1,3]])

for i in range(len(MALICIOUS_AGENTS)):
    agent = MALICIOUS_AGENTS[i]
    for j in range(NUM_ROBOTS):
            laplacian[j][agent] = K * laplacian[j][agent]
     



SAFE_REGION_EXPONENTIAL = np.exp(-BETA*SAFE_REGION) * np.ones((NUM_ROBOTS,NUM_ROBOTS))
#print(SAFE_REGION_EXPONENTIAL)

positions = np.random.randint(ROBOT_RADIUS,ROBOT_WINDOW_WIDTH-ROBOT_RADIUS,(NUM_ROBOTS,2))
screen = pygame.display.set_mode((WINDOW_WIDTH,WINDOW_HEIGHT))
screen.fill((255,255,255))

dt = 0.2

desired_position = np.random.randint(2*ROBOT_RADIUS,ROBOT_WINDOW_WIDTH-(2*ROBOT_RADIUS),(1,2))
old_positions = np.copy(positions)

while True:
    
    pygame.time.delay(10)

    #displacement = (velocity[0]*dt,velocity[1]*dt)
    #position = (position[0] + displacement[0],position[1]+displacement[1])
    screen.fill((255,255,255))
    new_velocity = -K_p * np.matmul(laplacian,positions) + b_matrix
      
    pairwise_distances = squareform( pdist(positions))
    SAFE_REGION_MATRIX = SAFE_REGION*np.eye(NUM_ROBOTS)
    pairwise_distances = pairwise_distances + SAFE_REGION_MATRIX
    pairwise_distances = np.clip(pairwise_distances,0,SAFE_REGION)
    pairwise_distances = np.exp(-BETA * pairwise_distances)
    difference = ALPHA * (pairwise_distances - SAFE_REGION_EXPONENTIAL)


    pairwise_repulsive_velocities = np.zeros((NUM_ROBOTS,NUM_ROBOTS,2))
    for i in range(NUM_ROBOTS):
        for j in range(NUM_ROBOTS):
            difference_vector = positions[i] - positions[j]
       
            normalized_distance = np.linalg.norm(difference_vector)
      

            if(normalized_distance>0):
                difference_vector = difference_vector/normalized_distance

            pairwise_repulsive_velocities[i,j,:] = difference[j][i] *difference_vector
  
    
    repulsive_velocity = np.sum(pairwise_repulsive_velocities,axis=1)
   
    centroid = 1.0/NUM_ROBOTS * np.sum(positions,axis=0,keepdims=True)
    desired_position_velocity = K_tracking_control * ( desired_position - centroid)
    
    tracking_control_velocity = np.repeat(desired_position_velocity,NUM_ROBOTS,axis=0)

    displacement = (new_velocity+repulsive_velocity+tracking_control_velocity)*dt   

    positions = positions + displacement


    mean_position = np.mean(positions,axis=0)
    

    velocity_direction = (new_velocity+repulsive_velocity + tracking_control_velocity) 
    norm = np.linalg.norm(velocity_direction,axis=1)
    velocity_vector = 15 *velocity_direction / norm[:,None]
    #fig, axes = plt.subplots(1, 1,)
    #if(trajectories[0] is not None):
    #    x, y = zip(*trajectories[0])
    #
    #    axes.plot(x, y, color='green', label='test')
    #
    #    fig.canvas.draw()
    #    screen.blit(fig, (ROBOT_WINDOW_WIDTH, 100))

    for i in range(len(positions)):
        pygame.draw.circle(screen,colors[i],positions[i].tolist(),ROBOT_RADIUS,width=3)
        pygame.draw.line(screen,(0,255,0),positions[i],positions[i] + velocity_vector[i],3)
        pygame.draw.line(screen,(0,0,0),[ROBOT_WINDOW_WIDTH,0], [ROBOT_WINDOW_WIDTH,ROBOT_WINDOW_HEIGHT])
        #pygame.draw.line(screen,colors[i],old_positions[i].tolist(),positions[i].tolist())
        trajectories[i][trajectory_index[i]] =  positions[i].tolist()
        trajectory_index[i] = (trajectory_index[i] + 1) % TRAJECTORY_SIZE

        for j in range(1,trajectory_index[i]):
            pygame.draw.line(screen,colors[i],trajectories[i][j],trajectories[i][j-1])

        #text_surface = my_font.render(str(positions[i]),False,colors[i])
        #screen.blit(text_surface,(ROBOT_WINDOW_WIDTH+50,i*20))

    
    pygame.draw.circle(screen,(0,0,0),desired_position[0].tolist(),4)
    pygame.display.update()
    
    if SLOW_DOWN:
        pygame.time.wait(50)

    
    for e in pygame.event.get():

        if e.type==pygame.MOUSEBUTTONUP:
            pos = pygame.mouse.get_pos()
            if(pos[0]>=0 and pos[0]<ROBOT_WINDOW_WIDTH and pos[1]>=0 and pos[1]<ROBOT_WINDOW_HEIGHT):
                desired_position = np.array([[pos[0],pos[1]]])
            
        if e.type==QUIT:
            pygame.quit()
            sys.exit()
