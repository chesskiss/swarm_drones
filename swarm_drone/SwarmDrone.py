from swarm import *
from tello import *
import keyboard
import time
import cv2
import numpy as np
import av
from threading import Thread, Lock
from vpython import *
from exstract_point import *
from find_location import *
import pandas as pd
from func_timeout import func_timeout

class BackgroundFrameReader:
    """
    This class read frames using PyAV in background. Use
    backgroundFrameRead.frame to get the current frame.
    """

    def __init__(self, address):
        self.address = address
        self.time=time.time()
        self.frame = None
        self.frames = []
        self.i=0
        self.take=False
        self.take_i=-100

        try:
            self.container = av.open(self.address, timeout=(8, None))
        except av.error.ExitError:
            try:
                self.container = av.open(self.address, timeout=(8, None))
            except av.error.ExitError:
                print('Failed to grab video frames from video stream')
                return 0
            
        self.read_freams = Thread(target=self.update_frame, args=(), daemon=True)
        self.read_freams.start()
        self.stopped = False
        
    def take_img(self):
        self.take=False
        self.take_i=self.i+8
        print("start take img, i=",self.i,"take_i=",self.take_i,"time=",time.time()-self.time)
        self.frames=[]
        self.frame = None
        self.stopped = False
        
        star_time=time.time()
        self.take=True
        while True:
                
                if self.frame is not None:
                    print(f"soccess take img i={self.i}, take_i={self.take_i} time={time.time()-self.time}")
                    img1=self.frame
                    self.frame=None
                    self.take=False
                    return img1
                
                if time.time()-star_time>5:
                        self.take=False
                        print("not soccess take img")
                        print(f"self.i=",self.i)
                        return None

    def update_frame(self):
        """Thread worker function to retrieve frames using PyAV
        Internal method, you normally wouldn't call this yourself.
        """
        try:
            for frame in self.container.decode(video=0):
                img= None
                img=frame.to_image()
                img=np.array(img)
                if img is not None:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    #if self.i%7==0:
                        #cv2.imwrite(f"./chekframe/img{self.i} time={time.time()-self.time}.png", img)

                if self.take and self.i>=self.take_i and self.i<=self.take_i+30:
                    print("change frame, i=",self.i,"take_i=",self.take_i,"time=",time.time()-self.time)
                    self.frame=img
                    self.take_i=-100
                
                self.i=self.i+1

                if self.stopped:
                    self.frame = None
                    break
                
        except Exception as e:
                print("An error occurred in the 'update_frame':", e)
                self.frame = None
        
    def stop(self):
        """Stop the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.stopped = True
        self.read_freams.join()
        self.container.close()

class SwarmDrones:
    def __init__(self,ip_list=None,points=None,v=20,start_pos_drone=None,sim=False
                 ,use_camera=True,output_path=None
                 ,green_rectangle_info=None):    
        
        self.output_path=output_path            
        self.use_camera=use_camera
        self.sim=sim
        self.v=v
        self.points=points
        self.max_error=2*sqrt((points[0][0]-points[1][0])**2+(points[0][1]-points[1][1])**2)

        self.run=True
        self.start_yaw=0
        self.start_time=time.time()
        self.timeout=150
        self.img_index=0
        self.tello_take_img=0
        self.real_location=[]   
        self.drone_speres=[]
        self.start_loc_in_curve_list=[]
        self.vp_points=[]
        self.vector_points=[]
        self.tello_yaw=[]

     

        self.num_drone=len(start_pos_drone)
        self.location=-1*np.ones(self.num_drone)

        # draw points in vpython
        for i,point in enumerate (self.points):
            vec_point=vector(point[0],point[1], 0)
            Vp_point = sphere(pos=vec_point, radius=4, color=color.red)
            self.vp_points.append(Vp_point)
            self.vector_points.append(vec_point) 

        # draw drones in vpython
        colors=[color.yellow,color.white,color.blue,color.green,color.orange,color.red,color.cyan,color.magenta]
        point_start_in_curve = [int(i * (len(points)/(self.num_drone))) for i in range(self.num_drone+1)]
        self.point_start_in_curve=point_start_in_curve[:-1]

        for i in range(self.num_drone):
            self.real_location.append(vector(start_pos_drone[i][0],start_pos_drone[i][1], 0))
            #self.display_loc[i].append((start_pos_drone[i][0],start_pos_drone[i][1]))
            
            drone_spere = sphere(pos=self.real_location[i], radius=5,make_trail=True, retain=500 , color=colors[i])
            self.drone_speres.append(drone_spere)
            self.start_loc_in_curve_list.append(vector(points[self.point_start_in_curve[i]][0],points[self.point_start_in_curve[i]][1], 0))
        
        if not sim:
            if ip_list is not None:
                self.swarm = TelloSwarm.fromIps(ip_list)
            else:
                self.swarm = TelloSwarm.find_tello(number_tello=self.num_drone)
            self.swarm.connect()

            for i, tello in enumerate (self.swarm):
                    print("tello battery=",tello.get_battery())
                    yaw=tello.get_yaw()
                    self.tello_yaw.append(yaw)
                    print(f"yaw of tello{i}=",yaw)
                    
        
            if  use_camera and green_rectangle_info is not None:

                self.green_width=green_rectangle_info['width']
                self.green_high=green_rectangle_info['high']
                self.green_loc=green_rectangle_info['location']

                address_schema = 'udp://@{ip}:{port}' 
                address = address_schema.format(ip="0.0.0.0", port=11111)
                #self.start_yaw=self.swarm.tellos[0].get_yaw()
                #print("self.start_yaw=",self.start_yaw)
                self.swarm.tellos[0].streamon()
                try:
                    self.frame_reader = BackgroundFrameReader(address)
                except:
                    print("can't take frame")
                    self.use_camera=False
                self.swarm.tellos[0].streamoff()
                time.sleep(0.1)
            #print("self.start_yaw=",self.start_yaw)
            print("self.use_camera=",self.use_camera)
    
              
    def move(self):
        if not self.sim:
            self.swarm.takeoff()
            self.swarm.move_up(40)
        number=0

        while self.run:
            if time.time()-self.start_time>self.timeout:
                break        
            if not self.sim:
                print("\nnumber = ",number)
                self.swarm.connect(wait_for_state=False)
                number=number+1 
                for i, queue in enumerate (self.swarm.funcQueues):
                    func=self.go_to_next_point(i)
                    queue.put(func)

                self.swarm.funcBarrier.wait()
                print("moving")
                self.swarm.funcBarrier.wait()

                if self.use_camera:
                    strarttime=time.time()
                    try:
                        func_timeout(10, self.take_frame())
                    except:
                        self.swarm.tellos[self.tello_take_img].send_command_without_return("streamoff")
                        self.tello_take_img=(self.tello_take_img+1)%self.num_drone
                        self.img_index=self.img_index+1
                    endtime=time.time()
                    print(f"take img time={endtime-strarttime}")
            else:
                for i in range(self.num_drone):
                    func=self.go_to_next_point(i)
        self.end()

    def go_to_next_point(self,i):
        loc=int(self.location[i])
        if loc==-1:
            next_loc=self.point_start_in_curve[i]
            x=self.points[next_loc][0]-self.real_location[i].x 
            y=self.points[next_loc][1]-self.real_location[i].y 
            self.location[i]=next_loc
            #self.display_loc[i].append((self.real_location[i].x,self.real_location[i].y))

            #print(f"i={i},x={x},y={y}")
            self.real_location[i]=self.real_location[i]+vector(x,y,0)
            self.drone_speres[i].pos=self.real_location[i]           
        else:
            next_loc=int((loc+1)%len(self.points))
            x=self.points[next_loc][0]-self.real_location[i].x  
            y=self.points[next_loc][1]-self.real_location[i].y  
            self.location[i]=next_loc
            self.real_location[i]=self.real_location[i]+vector(x,y,0)
            self.drone_speres[i].pos=self.real_location[i]
            #self.display_loc[i].append((self.real_location[i].x,self.real_location[i].y)) 
          
        
        if self.sim:
            time.sleep(1) 
        if sqrt(x**2+y**2)<20:       
            return lambda i,tello: tello.send_command_without_return("command")
        return lambda i,tello: tello.go_xyz_speed( round(x), round(y),0,self.v)

    def take_frame(self):
        for i,tello in enumerate (self.swarm):
            if i==self.tello_take_img:
                    img = None 
                    yaw=tello.get_yaw()
                    print("yaw=",yaw)
                    clockwise,size=self.calc_rotate(yaw,i)
                    if size>2:
                        if clockwise:
                            tello.rotate_clockwise(size)
                        else:
                            tello.rotate_counter_clockwise(size)
                    
                    tello.streamon()
                    img=self.frame_reader.take_img()
               
                    if img is not None:
                        # convert to rgb
                        cv2.imwrite(f"{self.output_path}/num img={self.img_index}, img from drone {i}.png", img)
                        print(f"take img from drone {i},num img={self.img_index} frame size={img.shape}")
                        a=find_location(self.green_width,self.green_high,img)
                        if a is not None:
                            location=[self.green_loc[0]-a[0],self.green_loc[1]+a[1],self.green_loc[2]+a[2]]
                            print("\n\n\n drone",i,"\nlocation = ",location, "\n  sim_location = ",self.real_location[i],", a=",a )
                            print("x dis=",location[0]-self.real_location[i].x,", y dis=",location[1]-self.real_location[i].y, "z high=",location[2],"\n\n\n")
                            if sqrt((location[0]-self.real_location[i].x)**2+(location[1]-self.real_location[i].y)**2)<self.max_error:
                                self.real_location[i].x=location[0]
                                self.real_location[i].y=location[1]
                                self.drone_speres[i].pos=self.real_location[i]
                                #self.display_loc[i].append((location[0],location[1]))
                            else:
                                print("can't find location, dis is to big")

                        else:
                            print("can't find location")
                        self.img_index=self.img_index+1
                    try:
                        tello.streamoff()
                    except:
                            try:
                                tello.streamoff()
                            except:
                                print("can't streamoff, shut down camera")
                                self.use_camera=False
                        
                    img = None 
     
        self.tello_take_img=(self.tello_take_img+1)%self.num_drone


    def end(self):
            print("land")
            if not self.sim:  
                self.swarm.land()
                self.swarm.end()
                if self.use_camera:
                    self.frame_reader.stop()
                        

    def calc_rotate(self,yaw,i):
            dis=yaw-self.tello_yaw[i]
            print("dis=",dis)
            if dis>=0 and dis <180:
                return False,dis
            elif dis>180:
                return True,360-dis 
            elif dis <0 and dis >-180:
                return True,-dis
            else:
                return False,360+dis
        
    '''
    def display_sim(self):
        # Create a new figure
        drone1=self.display_loc[0]
        drone2=self.display_loc[1]
        drone3=self.display_loc[2]
        plt.figure()
        
        # Plot the points from drone1 in red with labels
        for i, point in enumerate(drone1):
            x, y = point
            plt.scatter(x, y, color='red')
            plt.annotate(f' {i+1}', (x, y), textcoords="offset points", xytext=(10,0), ha='center')

        # Plot the points from drone2 in blue with labels
        for i, point in enumerate(drone2):
            x, y = point
            plt.scatter(x, y, color='blue')
            plt.annotate(f' {i+1}', (x, y), textcoords="offset points", xytext=(0,-10), ha='center')

        # Plot the points from drone3 in green with labels
        for i, point in enumerate(drone3):
            x, y = point
            plt.scatter(x, y, color='green')
            plt.annotate(f' {i+1}', (x, y), textcoords="offset points", xytext=(0,10), ha='center')
        
        for i, point in enumerate(self.points):
            x, y = point
            plt.scatter(x, y, color='black')
        # Add labels
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')

        # Show the plot
        plt.show()
    '''
