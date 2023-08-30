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

class BackgroundFrameReader:
    """
    This class read frames using PyAV in background. Use
    backgroundFrameRead.frame to get the current frame.
    """

    def __init__(self, address, with_queue = False):
        self.address = address
        self.lock = Lock()
        self.frame = None
        self.frames = []
        self.with_queue = with_queue
        self.dont_take_this_frame = False

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
        
        self.dont_take_this_frame=True
        self.frames=[]
        self.frame = None
        self.stopped = False
        
        star_time=time.time()
        while True:
                if len(self.frames) >= 2:
                    self.frame=self.frames[1]
                    return self.frame
                else:
                    if time.time()-star_time>5:
                        print("not soccess take img")
                        return None
                    
    def update_frame(self):
        """Thread worker function to retrieve frames using PyAV
        Internal method, you normally wouldn't call this yourself.
        """
        try:
            for frame in self.container.decode(video=0):
                if len (self.frames) <= 3:
                    self.frames.append(np.array(frame.to_image()))
                else:
                    self.frames=[]
                    self.frames.append(np.array(frame.to_image()))

                if self.stopped:
                    self.frame = None
                    break
                
        except Exception as e:
                print("An error occurred:", e)
                self.frame = None
        
    def stop(self):
        """Stop the frame update worker
        Internal method, you normally wouldn't call this yourself.
        """
        self.stopped = True
        self.read_freams.join()
        self.container.close()

class SwarmDrones:
    def __init__(self,ip_list=None,points=None,v=20,start_pos_drone=None,sim=False,use_camera=True,output_path=None):    
        
        self.output_path=output_path            
        self.use_camera=use_camera
        self.sim=sim
        self.v=v
        self.points=points

        self.real_width=2.97
        self.real_high=2.10
        self.img_index=0
        self.tello_take_img=0
        self.real_location=[]   
        self.drone_speres=[]
        self.start_loc_in_curve_list=[]
        self.vp_points=[]
        self.vector_points=[]

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
            drone_spere = sphere(pos=self.real_location[i], radius=5,make_trail=True, retain=50 , color=colors[i])
            self.drone_speres.append(drone_spere)
            self.start_loc_in_curve_list.append(vector(points[self.point_start_in_curve[i]][0],points[self.point_start_in_curve[i]][1], 0))
        
        if not sim:
            self.swarm = TelloSwarm.fromIps(ip_list)
            self.swarm.connect()
            self.conect=Thread(target=self.stay_conect,daemon=True)
            #conect.start()

            for tello in self.swarm:
                    print("tello battery=",tello.get_battery())
        
            if  use_camera:
                address_schema = 'udp://@{ip}:{port}' 
                address = address_schema.format(ip="0.0.0.0", port=11111)
                self.swarm.tellos[0].streamon()
                try:
                    self.frame_reader = BackgroundFrameReader(address, with_queue=False)
                except:
                    print("can't take frame")
                    self.use_camera=False
                self.swarm.tellos[0].send_command_without_return("streamoff")
                time.sleep(0.1)
    
              
    def move(self):
        if not self.sim:
            self.swarm.takeoff()
            self.swarm.move_up(20)
        number=0
        while True:        
            if not self.sim:
                print("\nnumber = ",number)
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
                        self.take_frame()
                    except:
                        self.tello_take_img=(self.tello_take_img+1)%self.num_drone
                    endtime=time.time()
                    print(f"take img time={endtime-strarttime}")
            else:
                for i in range(self.num_drone):
                    func=self.go_to_next_point(i)

            #if keyboard.is_pressed('q'):
            #    break
        
        self.end()
        print("end")

    def go_to_next_point(self,i):
        loc=int(self.location[i])
        if loc==-1:
            next_loc=self.point_start_in_curve[i]
            x=self.points[next_loc][0]-self.real_location[i].x 
            y=self.points[next_loc][1]-self.real_location[i].y 
            self.location[i]=next_loc
            
            print(f"i={i},x={x},y={y}")
            self.real_location[i]=self.real_location[i]+vector(x,y,0)
            self.drone_speres[i].pos=self.real_location[i]           
        else:
            next_loc=int((loc+1)%len(self.points))
            x=self.points[next_loc][0]-self.real_location[i].x  
            y=self.points[next_loc][1]-self.real_location[i].y  
            self.location[i]=next_loc
            self.real_location[i]=self.real_location[i]+vector(x,y,0)
            self.drone_speres[i].pos=self.real_location[i]          
            print(f"i={i},x={x},y={y}")
            print(f"i={i},next_loc={next_loc},loc={loc},real_location={self.real_location[i]}")
        if self.sim:
            time.sleep(0.1) 
        if sqrt(x**2+y**2)<20:       
            return lambda i,tello: tello.send_command_without_return("command")
        return lambda i,tello: tello.go_xyz_speed( round(x), round(y),0,self.v)

    def take_frame(self):
        for i,tello in enumerate (self.swarm):
            if i==self.tello_take_img:
                    img = None 
                    tello.streamon()
                    img=self.frame_reader.take_img()
               
                    if img is not None:
                        # convert to rgb
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        cv2.imwrite(f"{self.output_path}/num img={self.img_index}, img from drone {i}.png", img)
                        print(f"take img from drone {i},num img={self.img_index} frame size={img.shape}")
                        location=find_location(self.real_width,self.real_high,img)
                        print("location=",location)
                        print("x dis=",location[0]-self.real_location[i].x,", y dis=",location[1]-self.real_location[i].y)
                        self.img_index=self.img_index+1
                        tello.send_command_without_return("streamoff")
                        time.sleep(0.1)
                    img = None 
     
        self.tello_take_img=(self.tello_take_img+1)%self.num_drone

    def stay_conect(self):
        func=lambda i,tello: tello.send_command_without_return("command")
        for j, queue in enumerate (self.swarm.funcQueues):               
                    queue.put(func)
        time.sleep(7)

    def end(self):
        self.swarm.land()
        self.swarm.end()
        self.frame_reader.stop()
        

    def test_img(self):
        for i in range(10):
            starttime=time.time()
            self.take_frame()
            endtime=time.time()
            print(f"take img time={endtime-starttime}")
            time.sleep(2)


ip_list=[
    "192.168.187.90", # ריק
    "192.168.187.88", # צהוב
    "192.168.187.203" # לבן
    
    
    #"192.168.1.107", # צהוב
    #"192.168.1.106",  # לבן
    #"192.168.1.108"   # ריק
]

size=250
num_points=15
start_point=[0,0]
script_path = os.path.abspath(__file__)
folder_path = os.path.dirname(script_path)
input_path = os.path.join(folder_path, "curves_test/img1.png")
output_path = os.path.join(folder_path, "output")
img = cv2.imread(input_path)
binary_img=img_to_binary_img(img)

points=exstract_points(binary_img,size,start_point,num_points=num_points)
# convert to int
points=[[round(point[0]),round(point[1])] for point in points]
start_point=[(0,0),(120,0),(240,0)]
sim=False
#sim=True
use_camera=True
#use_camera=False
swarm=SwarmDrones(ip_list=ip_list,points=points,v=30,start_pos_drone=start_point,sim=sim,use_camera=use_camera,output_path=output_path)
swarm.img_index=72
#swarm.test_img()

swarm.move()
swarm.end()
