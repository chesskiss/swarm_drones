from SwarmDrone import *
from exstract_point import *
import cv2
import os


ip_list=  [
    
    #"192.168.23.90", # ריק
    #"192.168.23.88", # צהוב
    #"192.168.23.203" # לבן
    
#'192.168.10.1'
    
    "192.168.1.106", # צהוב
    "192.168.1.107",  # לבן
    "192.168.1.108"   # ריק
]
ip_list=None

script_path = os.path.abspath(__file__)
folder_path = os.path.dirname(script_path)
input_path = os.path.join(folder_path, "curves_test/img1.png")
output_path = os.path.join(folder_path, "output")
imgc = cv2.imread(input_path)
binary_img=img_to_binary_img(imgc)

curve_info={"num_points":9,"size":200,"start_point":[0,0],"img":binary_img,"resolution":0}  
green_rectangle_info={"width":29.7,"high":21,"location":[473,0,165]}

points=exstract_points(curve_info)
# convert to int
points=[[round(point[0]),round(point[1])] for point in points]
start_point=[(0,0),(80,0),(160,0)]
#start_point=[(0,0),(80,0)]
#start_point=[(0,0)]
sim=False
#sim=True
use_camera=True
#use_camera=False
swarm=SwarmDrones(ip_list=ip_list,points=points,v=40,start_pos_drone=start_point
                  ,sim=sim,use_camera=use_camera,output_path=output_path,
                  green_rectangle_info=green_rectangle_info)
swarm.img_index=0

swarm.move()
#swarm.display_sim()
print("end")
