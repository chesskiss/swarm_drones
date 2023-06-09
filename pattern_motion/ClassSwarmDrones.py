import numpy as np
from vpython import *
from exstract_point import *
import keyboard
from djitellopy import Tello

class Drone:
    def __init__(self, pos):
        self.point_pos = -1
        self.pos = pos
        self.sphere = sphere(pos=self.pos, radius=9, make_trail=True, retain=50 ,color=color.green)
        self.start=False
        self.drone=Tello()
        self.drone.connect()

    def takeoff(self):
        # takeoff the drone
        self.drone.takeoff()
        self.start=True

    def move_to(self, next_pos):
    # Move the drone to the next position

        direction = (next_pos - self.pos)
        self.drone.go_xyz_speed(int(direction.x),int(direction.y),int(direction.z),25)
        self.pos = next_pos
        self.sphere.pos = self.pos

    def land(self):
        # land the drone
        self.drone.land()
        

class SwarmDronesMove:
    def __init__(self, num_drone, start_pos_list, points):
        self.num_drone = num_drone
        self.num_point = len(points)
        self.start_pos_list = start_pos_list
        self.points = []
        self.drones = []
        self.start_mode=True
                
        # Calculate start points for each drone based on the number of points
        start_points = np.linspace(len(points),0,self.num_drone+1)
        start_points = start_points.astype(int)   
        self.start_points = start_points[1:] 

        # Create VPython spheres for each point
        for point in points:
            Vp_point = sphere(pos=vector(point[0],point[1], 0), radius=7, color=color.red)
            self.points.append(Vp_point) 

        for i in range(self.num_drone):
            drone = Drone(self.start_pos_list[i])
            drone.next_pos = self.points[0].pos
            self.drones.append(drone)

    def start_move(self):
    # Move the drones to their respective start points

        for idx , this_drone in enumerate(self.drones):
            in_the_start_point=False
            while (in_the_start_point==False):
                if keyboard.is_pressed('q'):
                    self.stop()
                    
                if (this_drone.start==False):
                    this_drone.takeoff()
                
                this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
                this_drone.move_to(self.points[this_drone.point_pos].pos)

                # Check if the drone has reached his start point
                if (this_drone.point_pos==self.start_points[idx]):
                    in_the_start_point=True
        self.start_mode=False

    def move_drones(self):
        #if push on kay 'q' stop the program
        if keyboard.is_pressed('q'):
            self.stop()

        for this_drone in self.drones:  #need to be simultanius
            this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
            this_drone.move_to(self.points[this_drone.point_pos].pos)

    def move(self):
        if (self.start_mode):  
            self.start_move()
        else:
            self.move_drones()

    def move_point(self,direction,v):
    # Move all points in the given direction

            for i in range(int(direction.mag/v)):
                rate(7)
                self.move_drones()
                for point in self.points:
                    point.pos=point.pos+v*direction

    def stop(self):
        # land all drones
        for this_drone in self.drones:
            this_drone.land()
        print("stop because of 'q'")
        exit()

