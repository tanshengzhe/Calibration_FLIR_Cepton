

from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import os
import numpy as np
import open3d as o3d


from sensor_msgs.msg import PointCloud2, PointField


# from cepton_messages.msg import CeptonPointData
import sys

from registry import converts_from_numpy, converts_to_numpy

import time

from threading import Thread, Lock

DUMMY_FIELD_PREFIX = '__'

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')),
                 (PointField.UINT8, np.dtype('uint8')),
                 (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')),
                 (PointField.INT32, np.dtype('int32')),
                 (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')),
                 (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

@converts_to_numpy(PointField, plural=True)
def fields_to_dtype(fields, point_step):
    '''Convert a list of PointFields to a numpy record datatype.
    '''
    offset = 0
    np_dtype_list = []
    for f in fields:
        while offset < f.offset:
            # might be extra padding between fields
            np_dtype_list.append(
                ('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        dtype = pftype_to_nptype[f.datatype]
        if f.count != 1:
            dtype = np.dtype((dtype, f.count))

        np_dtype_list.append((f.name, dtype))
        offset += pftype_to_nptype[f.datatype].itemsize * f.count

    # might be extra padding between points
    while offset < point_step:
        np_dtype_list.append(('%s%d' % (DUMMY_FIELD_PREFIX, offset), np.uint8))
        offset += 1

    return np_dtype_list


@converts_from_numpy(PointField, plural=True)
def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = int(np.prod(shape))
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

@converts_to_numpy(PointCloud2)
def pointcloud2_to_array(cloud_msg, squeeze=True):
    ''' Converts a rospy PointCloud2 message to a numpy recordarray

    Reshapes the returned array to have shape (height, width), even if the
    height is 1.

    The reason for using np.frombuffer rather than struct.unpack is
    speed... especially for large point clouds, this will be <much> faster.
    '''
    # construct a numpy record type equivalent to the point type of this cloud
    dtype_list = fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)

    # parse the cloud into an array
    cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

    # remove the dummy fields that were added
    cloud_arr = cloud_arr[
        [fname for fname, _type in dtype_list if not (
            fname[:len(DUMMY_FIELD_PREFIX)] == DUMMY_FIELD_PREFIX)]]

    if squeeze and cloud_msg.height == 1:
        return np.reshape(cloud_arr, (cloud_msg.width,))
    else:
        return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

def get_xyz_points(cloud_array, remove_nans=True, dtype=np.float):
    '''Pulls out x, y, and z columns from the cloud recordarray, and returns
    a 3xN matrix.
    '''
    # remove crap points
    if remove_nans:
        mask = np.isfinite(cloud_array['x']) & \
               np.isfinite(cloud_array['y']) & \
               np.isfinite(cloud_array['z']) & \
               np.isfinite(cloud_array['intensity'])
            
        cloud_array = cloud_array[mask]

    # pull out x, y, and z values
    points = np.zeros(cloud_array.shape + (4,), dtype=dtype)
    points[...,0] = cloud_array['x']
    points[...,1] = cloud_array['y']
    points[...,2] = cloud_array['z']
    points[...,3] = cloud_array['intensity']
    # time_us = cloud_array['timestamp_us'][-1]
    # time_s = cloud_array['timestamp_s'][-1]
    # print("timestamp_us: ",str(time_s)+'.'+str(time_us))
    return points

def pointcloud2_to_xyz_array(cloud_msg, remove_nans=True):
    return get_xyz_points(
        pointcloud2_to_array(cloud_msg), remove_nans=remove_nans)

class message_filter():

    def __init__(self):
        
        self.sensor_data = {}
        self.sensor_time = {}
        self.flag = False
        self.br = CvBridge()

    def _compare_time(self, Camera_time,Lidar_time):
    
        indx = np.argmin(abs(np.array(Camera_time)-Lidar_time))
        
        return indx

    # def get_latest_msg_v2(self):

    #     topic_lsit = self.sensor_time.keys()
        
    #     lidar_time = self.sensor_time['cepton_pcl2']
    #     img_time = self.sensor_time['video_frames']
        

    #     n_t = lidar_time[9]
    #     img_idx = self._compare_time(img_time,n_t) 


    #     lidar_msg = self.sensor_data['cepton_pcl2'][9]
    #     img_msg = self.sensor_data['video_frames'][img_idx]

    #     points = pointcloud2_to_xyz_array(lidar_msg)
    #     pcd_as_numpy_array = np.array(points)
    #     pcd_as_numpy_array = pcd_as_numpy_array[:,:4]
    #     mask = pcd_as_numpy_array[:,1]<120
    #     pcd_as_numpy_array = pcd_as_numpy_array[mask,:]

    #     current_pcd = pcd_as_numpy_array
    #     current_frame = self.br.imgmsg_to_cv2(img_msg)


    #     return current_pcd,current_frame

    def get_latest_msg(self,topic1,topic2,offset = 0):

        
        lidar_time = self.sensor_time[topic1]
        img_time = self.sensor_time[topic2]
        

        n_t = lidar_time[8]
        img_idx = self._compare_time(img_time,n_t) 


        lidar_msg = self.sensor_data[topic1][8]

        img_idx = img_idx + offset
        if img_idx > 9:
            img_idx = 9

        img_msg = self.sensor_data[topic2][img_idx]

        points = pointcloud2_to_xyz_array(lidar_msg)
        pcd_as_numpy_array = np.array(points)
        pcd_as_numpy_array = pcd_as_numpy_array[:,:4]
        #mask = pcd_as_numpy_array[:,1]<200
        #pcd_as_numpy_array = pcd_as_numpy_array[mask,:]

        current_pcd = pcd_as_numpy_array
        current_frame = self.br.imgmsg_to_cv2(img_msg)
        if topic2 == '/blackfly_0/image_raw':
            current_frame = cv2.cvtColor(current_frame,cv2.COLOR_BayerBG2BGR)


        return current_pcd,current_frame,n_t

    def update(self, topic,msg):
        time = int(msg.header.stamp.sec)+int(msg.header.stamp.nanosec)*1e-9

        if topic not in self.sensor_data:
            self.sensor_data[topic] = []
            self.sensor_time[topic] = []
        
        if len(self.sensor_data[topic]) <10:
            self.sensor_data[topic].append(msg)
            
            self.sensor_time[topic].append(time)
   
        else:
            
            self.sensor_data[topic].pop(0)
            self.sensor_data[topic].append(msg)

            self.sensor_time[topic].pop(0)
            self.sensor_time[topic].append(time)
        if len(self.sensor_data['cepton_pcl2']) >9:
            self.flag = True 
        