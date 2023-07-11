import numpy as np
from vpython import *
import cv2
from exstract_point import *
import keyboard
from djitellopy import Tello
from ClassSwarmDrones import *

def test1():
    start_pos_drone=[vector(0,4,0),vector(0,2,0),vector(0,0,0),vector(0,-2,0),vector(0,-4,0)]
    num_drone=len(start_pos_drone)           
        
    img = cv2.imread(f"curves-test/img0.png")
    binary_img=img_to_binary_img(img)

    resolution=0.05 
    size=40
    start_point=[10,0]
    

    points=exstract_points(binary_img,size,start_point,resolution=resolution)
    swarm_drones = SwarmDronesMove(num_drone, start_pos_drone, points)

    i=0
    dir1=vector(0,5,0)
    dir2=vector(5,0,0)
    v=0.1

    while True:
        rate(7)
        swarm_drones.move()
        i+=1
        if (i>20):
            swarm_drones.move_point(dir1,v)
            swarm_drones.move_point(dir2,v)
            dir1=-1*dir1
            dir2=-1*dir2

def test2():
    start_pos_drone=[vector(0,0,0)]
    num_drone=1
    points=[[30,0],[100,70],[100,-70]]
    swarm_drones = SwarmDronesMove(num_drone, start_pos_drone, points)
    swarm_drones.start_move()
    while True:
        #rate() 
        swarm_drones.move()

def test3():
    start_pos_drone=[vector(0,0,0)]
    num_drone=1
    resolution=0.05 
    size=180
    num_points=8
    start_point=[20,0]
    img = cv2.imread(f"curves-test/img4.png")
    binary_img=img_to_binary_img(img)

    points=exstract_points(binary_img,size,start_point,num_points=num_points)
    swarm_drones = SwarmDronesMove(num_drone, start_pos_drone, points)
    swarm_drones.start_move()
    while True:
        swarm_drones.move()
test3()

