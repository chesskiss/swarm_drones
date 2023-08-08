import numpy as np
from vpython import *
from exstract_point import *
import keyboard
from swarm import *
import time

swarm_sdk = Swarm()

class Drone:
    def __init__(self, pos, id, v, sim=False):
        self.point_pos = -1
        self.pos = pos
        self.sphere = sphere(pos=self.pos, radius=9, make_trail=True, retain=50 ,color=color.green)
        self.start=False
        self.id = id
        self.sim = sim
        self.v = v

        print('checkpoint id = ', self.id)


    def takeoff(self):
        if not self.sim:
            # takeoff the drone
            print('checkpoint takeoff')
            swarm_sdk.commands = [str(self.id) + '>takeoff']
            swarm_sdk.start()

    def move_to(self, next_pos):
    # Move the drone to the next position
        direction = (next_pos - self.pos)
        if not self.sim:
            swarm_sdk.commands = [str(self.id) + '>go ' + \
                        str(direction.x) + ' ' + \
                        str(direction.y) + ' ' + \
                        str(direction.z) +' ' + str(self.v)]
             
            swarm_sdk.start()
        self.pos = next_pos
        self.sphere.pos = self.pos
        if self.sim:
            time.sleep(0.5)
    
    def land(self):
        if not self.sim:    
            # land the drone
            swarm_sdk.commands = [str(self.id) + '>land']
            swarm_sdk.start()
        

class SwarmDronesMove:
    def __init__(self, num_drone, start_pos_list, points,num_sim_drone=0,v=20):
        self.num_drone = num_drone+num_sim_drone
        self.num_point = len(points)
        self.start_pos_list = start_pos_list
        self.points = []
        self.drones = []
        self.start_mode=True
        self.v = v

        swarm_sdk.commands = ['scan ' + str(num_drone)]
        swarm_sdk.start()

        # Calculate start points for each drone based on the number of points
        start_points = np.linspace(len(points),0,self.num_drone+1)
        start_points = start_points.astype(int)   
        self.start_points = start_points[1:] 

        # Create VPython spheres for each point
        for point in points:
            Vp_point = sphere(pos=vector(point[0],point[1], 0), radius=7, color=color.red)
            self.points.append(Vp_point) 
            
        for i in range(num_drone):
            print('first for = ', i)
            drone = Drone(self.start_pos_list[i], i+1, self.v)
            drone.next_pos = self.points[0].pos
            self.drones.append(drone)
        
        for j in range(num_drone,num_drone+num_sim_drone):
            #print(' for = ', j+1)
            drone = Drone(self.start_pos_list[num_drone+j], -1 ,self.v, sim=True)
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
        while(True):
            if keyboard.is_pressed('q'):
                self.stop()

            if (self.start_mode):  
                self.start_move()
            else:
                self.move_drones()

    def find_true_point(self):
        for this_drone in self.drones:
            if not(this_drone.sim):
                this_drone.find_location()
            


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

