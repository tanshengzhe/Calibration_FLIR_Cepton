import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from rclpy.executors import MultiThreadedExecutor
import os
import numpy as np
import open3d as o3d

import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs


from sensor_msgs.msg import PointCloud2, PointField


# from cepton_messages.msg import CeptonPointData
import sys

#from registry import converts_from_numpy, converts_to_numpy

import time

from threading import Thread, Lock
from Msg_Filter import *


msg_filter = message_filter()



class ImageListener(Node):

    def __init__(self):
        super().__init__('data_subsriber_node')

        self.topic = '/blackfly_0/image_raw'
        #sensor_msgs.PointCloud2
        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`

        self.subscription = self.create_subscription(
            Image, 
            self.topic , 
            self.img_callback, 
            1)

    def img_callback(self, msg):
        """
        Callback function.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving video frame')

        msg_filter.update(self.topic,msg)
  
    
class PonintCloudListener(Node):

    def __init__(self):
        super().__init__('data_subsriber_node2')
     
        #sensor_msgs.PointCloud2
        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`

        self.topic = 'cepton_pcl2'
        self.pcd_subscriber = self.create_subscription(
            PointCloud2,    # Msg type
            self.topic ,  # topic
            self.listener_callback,      # Function to call
            1                          # QoS
        )

                
    def listener_callback(self, msg):


        # print(msg.header)
        msg_filter.update(self.topic,msg)


        t1_start = time.perf_counter()


class Test():
    def __init__(self,) :
        cwd = os.getcwd()
        target_file = "calib_May19"
        self.cam1_path = os.path.join(cwd,target_file,"cam1_img/")
        self.lidar_path = os.path.join(cwd,target_file,"lidar/")
        self.time_path = os.path.join(cwd,target_file,"timestamp/")
        self.id = 0

        if os.path.exists(os.path.join(cwd,target_file)) != True:
            os.mkdir(os.path.join(cwd,target_file))
        if os.path.exists(self.cam1_path) != True:
            os.mkdir(self.cam1_path)
        if os.path.exists(self.lidar_path) != True:
            os.mkdir(self.lidar_path)
        if os.path.exists(self.time_path) != True:
            os.mkdir(self.time_path)


    def start(self) :
       
        self.thread = Thread(target=self._update, args=())
        self.thread.start()
   

    def _update(self) :

        while True:
            if msg_filter.flag:
                points,img,time_stamp = msg_filter.get_latest_msg('cepton_pcl2','/blackfly_0/image_raw')
                
                # pcd = o3d.t.geometry.PointCloud(device)
                # pcd.point["positions"] = o3d.core.Tensor(points[:,:3],dtype, device)
                # pcd.point["intensities"] = o3d.core.Tensor(points[:,3], dtype, device)

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:,:3])
                intensity = np.array([points[:,3],points[:,3],points[:,3]]).T
                pcd.colors = o3d.utility.Vector3dVector(intensity)
                
                cv2.imshow("img", img)

                key = cv2.waitKey(20)

                #key=ord('g')

                if key == ord('g'):
                
                    o3d.io.write_point_cloud(os.path.join(self.lidar_path, str(self.id)+".pcd"),pcd)
                    
                    cv2.imwrite(os.path.join(self.cam1_path, str(self.id)+".png"), img)

                    np.save(os.path.join(self.time_path, str(self.id)+".npy"), time_stamp)
                    
                    print("save data ",self.id)
                    self.id+=1


grab_test = Test()
grab_test.start()

def main(args=None):

  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  listener1 = ImageListener()

  listener2 = PonintCloudListener()
  
  executor = MultiThreadedExecutor()
  executor.add_node(listener1)
  executor.add_node(listener2)


  executor.spin()

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  listener1.destroy_node()
  listener2.destroy_node()

  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()