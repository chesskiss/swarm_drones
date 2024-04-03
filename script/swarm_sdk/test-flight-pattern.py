import numpy as np
from vpython import *
import cv2
from exstract_point import *
import keyboard
from ClassSwarmDrones import *
from swarm_sdk.firebase_f import get_shape


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

def check_point(points):
    # chek if the distance between point and the next point is less than 20
    for i in range(len(points)-1):
        if (np.linalg.norm(np.array(points[i])-np.array(points[i+1]))<20):
            return False
    if (np.linalg.norm(np.array(points[0])-np.array(points[-1]))<20):
        return False
    return True

def test3():
    start_pos_drone=[vector(0,0,0)]
    num_drone=1
    num_sim_drone=0
    resolution=0.05 
    size=100
    num_points=7
    start_point=[20,0]
    #img_path = os.path.expandvars("$REPO/swarm_sdk/curves_test/img1.png") TODO
    img = cv2.imread(f"./curves_test/img1.png")
    binary_img=img_to_binary_img(img)

    points=exstract_points(binary_img,size,start_point,num_points=num_points)
    if (check_point(points)):
        swarm_drones = SwarmDronesMove(num_drone, start_pos_drone, points,num_sim_drone=num_sim_drone)
        print('before start move')
        swarm_drones.start_move()
        print('after start move')
        swarm_drones.move()

def firebase():
    shape = get_shape()
    start_pos_drone=[vector(0,0,0)]
    num_drone=1
    num_sim_drone=0
    resolution=0.05 
    size=100
    num_points=7
    start_point=[20,0]
    #img_path = os.path.expandvars("$REPO/swarm_sdk/curves_test/img1.png") TODO

    img = cv2.imread(f"./curves_test/{shape}.png")
    binary_img=img_to_binary_img(img)

    points=exstract_points(binary_img,size,start_point,num_points=num_points)
    if (check_point(points)):
        swarm_drones = SwarmDronesMove(num_drone, start_pos_drone, points,num_sim_drone=num_sim_drone)
        print('before start move')
        swarm_drones.start_move()
        print('after start move')
        swarm_drones.move()


if __name__ == '__main__':
    test3()

