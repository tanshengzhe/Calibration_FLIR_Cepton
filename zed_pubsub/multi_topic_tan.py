from std_msgs.msg import Header, Int16MultiArray, Float32MultiArray, Int64MultiArray
import numpy as np
import os
from rclpy.node import Node
import rclpy
import time
# from .package import globaldata
# from .package import globaldata
from multiprocessing import Process, Value, Array
import multiprocessing
from multiprocessing import Manager
from gps_msgs.msg  import GPSFix
from threading import Thread, Lock
import message_filters
from message_filters import ApproximateTimeSynchronizer, Subscriber
import copy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2, PointField
import open3d as o3d
import cv2 # OpenCV library
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from .Msg_Filter import pointcloud2_to_xyz_array





class multi_topic_tan(Node):


    def __init__(self):
        super().__init__('multi_topic_tan')
        self.br = CvBridge()


        cwd = os.getcwd()
        target_file = "calib_May19"
        self.cam1_path = os.path.join(cwd,target_file,"cam1_img/")
        self.cam2_path = os.path.join(cwd,target_file,"cam2_img/")
        self.cam3_path = os.path.join(cwd,target_file,"cam3_img/")
        self.lidar_path = os.path.join(cwd,target_file,"lidar/")

        # self.time_path = os.path.join(cwd,target_file,"timestamp/")
        self.id = 0

        if os.path.exists(os.path.join(cwd,target_file)) != True:
            os.mkdir(os.path.join(cwd,target_file))
        if os.path.exists(self.cam1_path) != True:
            os.mkdir(self.cam1_path)
        if os.path.exists(self.cam2_path) != True:
            os.mkdir(self.cam2_path)
        if os.path.exists(self.cam3_path) != True:
            os.mkdir(self.cam3_path)
        if os.path.exists(self.lidar_path) != True:
            os.mkdir(self.lidar_path)
        # if os.path.exists(self.time_path) != True:
        #     os.mkdir(self.time_path)

        self.flag=False
        self.queue_flag = False

        self.sensor_data = {}
        self.sensor_data['camera_0'] = []
        self.sensor_data['camera_1'] = []
        self.sensor_data['camera_2'] = []
        self.sensor_data['lidar'] = []


        #+++++++++++++++++++++++
        # self.perce_msg=message_filters.Subscriber(self, CAN, "/perception_msg")
        # self.AVState_msg=message_filters.Subscriber(self, Int64MultiArray, "/scoring_can") #The Subscriber for your code.
        self.camera_0_msg = message_filters.Subscriber(self, Image, "/blackfly_0/image_raw")
        self.camera_1_msg = message_filters.Subscriber(self, Image, "/blackfly_1/image_raw")
        self.camera_2_msg = message_filters.Subscriber(self, Image, "/blackfly_2/image_raw")
        self.lidar_msg = message_filters.Subscriber(self, PointCloud2, "/cepton_pcl2")

        # self.perce_msg=message_filters.Subscriber(self, NovatelHeading2, "/heading2") #The first topic I want to subscribe
        # self.AVState_msg=message_filters.Subscriber(self, GPSFix, "/gps") #The Second topic I want to subscribe
        # self.subscription = self.create_subscription(
        #     Image, 
        #     self.topic , 
        #     self.img_callback, 
        #     1)
        
        # self.topic = 'cepton_pcl2'
        # self.pcd_subscriber = self.create_subscription(
        #     PointCloud2,    # Msg type
        #     self.topic ,  # topic
        #     self.listener_callback,      # Function to call
        #     1                          # QoS
        # )

        queue_size = 1000
        ts_tolerance = 10

        ats = ApproximateTimeSynchronizer([self.camera_0_msg, self.camera_1_msg, self.camera_2_msg, self.lidar_msg], queue_size, ts_tolerance)
        ats.registerCallback(self.sensor_callback)

    def sensor_callback(self, camera_0_msg, camera_1_msg, camera_2_msg, lidar_msg):
        print(1)


           
        
        if len(self.sensor_data['lidar']) <10:
            

            self.sensor_data['camera_0'].append(camera_0_msg)
            self.sensor_data['camera_1'].append(camera_1_msg)
            self.sensor_data['camera_2'].append(camera_2_msg)
            self.sensor_data['lidar'].append(lidar_msg)
            
   
        else:
            self.sensor_data['camera_0'].pop(0)
            self.sensor_data['camera_0'].append(camera_0_msg)
            self.sensor_data['camera_1'].pop(0)
            self.sensor_data['camera_1'].append(camera_1_msg)
            self.sensor_data['camera_2'].pop(0)
            self.sensor_data['camera_2'].append(camera_2_msg)
            self.sensor_data['lidar'].pop(0)
            self.sensor_data['lidar'].append(lidar_msg)

        if len(self.sensor_data['lidar']) >9:
            self.queue_flag = True


        if self.flag!=True:
            self.flag=True
            self.start()








    def start(self) :
       
        self.thread = Thread(target=self._update, args=())
        self.thread.start()
   

    def _update(self) :

        while True:
            if self.queue_flag:
                print(2)
            
            # points,img,time_stamp = msg_filter.get_latest_msg('cepton_pcl2','/blackfly_0/image_raw')
            
            # pcd = o3d.t.geometry.PointCloud(device)
            # pcd.point["positions"] = o3d.core.Tensor(points[:,:3],dtype, device)
            # pcd.point["intensities"] = o3d.core.Tensor(points[:,3], dtype, device)
                points = self._pclconvert(self.sensor_data['lidar'][8])
                img1 = self._imgconvert(self.sensor_data['camera_0'][8])
                img2 = self._imgconvert(self.sensor_data['camera_1'][8])
                img3 = self._imgconvert(self.sensor_data['camera_2'][8])

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:,:3])
                intensity = np.array([points[:,3],points[:,3],points[:,3]]).T
                pcd.colors = o3d.utility.Vector3dVector(intensity)

                # print(img1)
                cv2.imshow("img", img2)
                key = cv2.waitKey(50)


                #key=ord('g')

                if key == ord('g'):
                
                    o3d.io.write_point_cloud(os.path.join(self.lidar_path, str(self.id)+".pcd"),pcd)
                    
                    cv2.imwrite(os.path.join(self.cam1_path, str(self.id)+".png"), img1)
                    cv2.imwrite(os.path.join(self.cam2_path, str(self.id)+".png"), img2)
                    cv2.imwrite(os.path.join(self.cam3_path, str(self.id)+".png"), img3)

                    # np.save(os.path.join(self.time_path, str(self.id)+".npy"), time_stamp)
                    
                    print("save data ",self.id)
                    self.id+=1



    def _imgconvert(self,frame_raw):
        current_frame = self.br.imgmsg_to_cv2(frame_raw,'bgr8')
        # current_frame = cv2.cvtColor(current_frame,cv2.COLOR_BayerBG2BGR)
        return current_frame
    
    def _pclconvert(self,point_raw):
        points = pointcloud2_to_xyz_array(point_raw)
        pcd_as_numpy_array = np.array(points)
        pcd_as_numpy_array = pcd_as_numpy_array[:,:4]
        #mask = pcd_as_numpy_array[:,1]<200
        #pcd_as_numpy_array = pcd_as_numpy_array[mask,:]

        current_pcd = pcd_as_numpy_array
        return current_pcd







def main(args=None):
    rclpy.init(args=args)
    # globaldata.init()



    minimal_subscriber = multi_topic_tan()

    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.spin()
    # rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()