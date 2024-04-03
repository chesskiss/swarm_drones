import numpy as np
from vpython import *
from exstract_point import *
from swarm import *
from find_location import *
import time
import subprocess
import os
import shutil


swarm_sdk = Swarm()
log_fpath = None

class Drone:
    def __init__(self, pos, id, v, sim=False):
        self.point_pos = -1
        self.pos = pos
        self.sphere = sphere(pos=self.pos, radius=9, make_trail=True, retain=50 ,color=color.green)
        self.start=False
        self.id = id
        self.sim = sim
        self.v = v
        print('checkpoint init Drone number ', self.id)


    def takeoff(self):
        #self.streamon()
    
        print('checkpoint takeoff of ', self.id-1)
        if not self.sim:
            # takeoff the drone
            swarm_sdk.commands = [str(self.id) + '>takeoff']
            swarm_sdk.start()
            

    def move_to(self, next_pos):
        self.get_frame()
    # Move the drone to the next position
        direction = (next_pos - self.pos)
        if not self.sim:
            swarm_sdk.commands = [str(self.id) + '>go ' + \
                        str(direction.x) + ' ' + \
                        str(direction.y) + ' ' + \
                        str(direction.z) + ' ' + str(self.v)]
             
            swarm_sdk.start()
        self.pos = next_pos
        self.sphere.pos = self.pos
        if self.sim:
            time.sleep(0.5)


    def get_frame(self) :
        swarm_sdk.commands = [str(self.id) + '>streamon']
        swarm_sdk.start()
    
        command = ['ffmpeg',
            '-i',               # Input option
            'udp://0.0.0.0:11111',  # Input source
            '-r',               # Output frame rate option
            '0.1',              # Output frame rate
            './drone_stream%d.png'.format(self.id) # Output image file pattern
        ]
        subprocess.run(command)
    
        swarm_sdk.commands = [str(self.id) + '>streamoff']
        swarm_sdk.start()
    

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
        self.GRREEN_WIDTH=10
        self.GREEN_HIGH=8

        #creating a log file
        dpath = './log'
        SwarmUtil.create_dir(dpath)
        start_time = str(time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(time.time())))
        global log_fpath
        log_fpath = f'{dpath}/{start_time}.txt'
        with open(log_fpath, 'w') as out:
            out.write(f'Hurray! Here we start\n')

        #Finding the drones
        swarm_sdk.commands = ['scan ' + str(num_drone), 'battery_check 20']
        swarm_sdk.start()

        # Calculate start points for each drone based on the number of points
        start_points = np.linspace(len(points),0,self.num_drone+1)
        start_points = start_points.astype(int)   
        self.start_points = start_points[1:] 

        # Create VPython spheres for each point
        for point in points:
            Vp_point = sphere(pos=vector(point[0],point[1], 0), radius=7, color=color.red)
            self.points.append(Vp_point) 
            
        for i in range(num_drone+num_sim_drone):
            if i < num_drone:
                print('drone num = ', i)
                drone = Drone(self.start_pos_list[i], i+1, self.v)
                drone.next_pos = self.points[0].pos
                self.drones.append(drone)
        
            else:
                print(' sim drone = ', i)
                drone = Drone(self.start_pos_list[i], -1 ,self.v, sim=True)
                drone.next_pos = self.points[0].pos
                self.drones.append(drone)


    def start_move(self):
    # Move the drones to their respective start points

        for idx , this_drone in enumerate(self.drones):
            in_the_start_point=False
            while (in_the_start_point==False):
                    
                if (this_drone.start==False):
                    this_drone.takeoff() 
                    this_drone.start=True

                this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
                this_drone.move_to(self.points[this_drone.point_pos].pos)

                # Check if the drone has reached his start point
                if (this_drone.point_pos==self.start_points[idx]):
                    in_the_start_point=True
        self.start_mode=False


    def move_drones(self):

        print("start move drones")
        for this_drone in self.drones:  #need to be simultanius
            this_drone.point_pos = (this_drone.point_pos+1) % self.num_point
            this_drone.move_to(self.points[this_drone.point_pos].pos)


    def move(self):
        while(True):
            
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

