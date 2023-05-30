import numpy as np
from vpython import *
import cv2
from exstract_point import *

class Drone:
    def __init__(self, pos):
        self.point_pos = -1
        self.pos = pos
        self.sphere = sphere(pos=self.pos, radius=1, make_trail=True, retain=100 ,color=color.green)

    def move_to(self, next_pos):
        direction = (next_pos - self.pos)
        # fly_to_xyz(direction.x,direction.y,direction.z)
        self.pos = next_pos
        self.sphere.pos = self.pos

class SwarmDrones:
    def __init__(self, num_drone, start_pos_list, points):
        self.num_drone = num_drone
        self.num_point = len(points)
        self.start_pos_list = start_pos_list
        self.points = []
        self.drones = []
        self.start_mode=True
        start_points = np.linspace(len(points),0,self.num_drone+1)
        start_points = start_points.astype(int)   
        self.start_points = start_points[1:] 

        for i in range(len(points)):
            point = sphere(pos=vector(points[i][0],points[i][1], 0), radius=0.5, color=color.red)
            self.points.append(point) 

        for i in range(self.num_drone):
            drone = Drone(self.start_pos_list[i])
            drone.next_pos = self.points[0].pos
            self.drones.append(drone)

    def start_move(self):
        for idx , this_drone in enumerate(self.drones):
            in_the_start_point=False
            while (in_the_start_point==False):
                rate(7)
                this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
                this_drone.move_to(self.points[this_drone.point_pos].pos)
                if (this_drone.point_pos==self.start_points[idx]):
                    in_the_start_point=True
        self.start_mode=False

    def move_drones(self):
        for this_drone in self.drones:  #need to be simultanius
            this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
            this_drone.move_to(self.points[this_drone.point_pos].pos)

    def move(self):
        if (self.start_mode):  
            self.start_move()
        else:
            self.move_drones()

    def move_point(self,director,v):
            for i in range(int(director.mag/v)):
                rate(7)
                self.move_drones()
                for point in self.points:
                    point.pos=point.pos+v*director


start_pos_drone=[vector(0,4,0),vector(0,2,0),vector(0,0,0),vector(0,-2,0),vector(0,-4,0)]
num_drone=len(start_pos_drone)           
     
img = cv2.imread(f"curves-test/img0.png")
binary_img=img_to_binary_img(img)

resolution=0.05 
size=40
start_point=[10,0]

points=exstract_points(binary_img,size,start_point,resolution=resolution)
swarm_drones = SwarmDrones(num_drone, start_pos_drone, points)

i=0
dir1=vector(1,5,0)
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
