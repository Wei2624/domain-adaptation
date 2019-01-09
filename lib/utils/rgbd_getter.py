#! /usr/bin/env python
import sys
sys.path.insert(0,'/home/weizhang/Documents/domain-adaptation/')
import path
import rospy
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
from kinect_rgdb_collector.srv import GetRGBD, GetRGBDRequest
import cv2
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
import sensor_msgs.msg
import threading
import sensor_msgs.point_cloud2 as pc2
import pcl
import tf
import collections
import os
from cv_bridge import CvBridge, CvBridgeError
import signal
import time
import ros_numpy
import operator
import pickle

# get_rgbd_service_name = 'get_rgbd_service'
# rospy.wait_for_service(get_rgbd_service_name)
# get_rgbd_service_session = rospy.ServiceProxy(get_rgbd_service_name, GetRGBD)
#
#
# def get_rgbd():
#     response = get_rgbd_service_session(GetRGBDRequest())
#
#     bridge = CvBridge()
#     rgb_image = bridge.imgmsg_to_cv2(response.rgb_image, desired_encoding="rgb8")
#     depth_image = bridge.imgmsg_to_cv2(response.depth_image, desired_encoding="passthrough")
#     return rgb_image, depth_image

def cloud_msg_to_np(msg):
    """
    Take a ros pointclud message and convert it to
    an nx3 numpy ndarray.
    :type msg: sensor_msg.msg.PointCloud2
    :rtype numpy.ndarray
    """
    pc = ros_numpy.numpify(msg)

    num_pts = reduce(operator.mul, pc.shape, 1)

    points = np.zeros((num_pts, 3))

    points[:, 0] = pc['x'].reshape((num_pts))

    points[:, 1] = pc['y'].reshape((num_pts))

    points[:, 2] = pc['z'].reshape((num_pts))

    return np.array(points, dtype=np.float32)


_rgbd_data_collector = None

RGBDAggregate = collections.namedtuple('RGBDAggregate', ['rgb_image', 'depth_image', 'camera_info','point_cloud_array', 'point_cloud_pcl'])

class RGBDDataCollector:
    def __init__(self, rgb_image_topic, depth_image_topic, camera_info_topic, cloud_topic):
        if not rospy.core.is_initialized():
            rospy.init_node('get_rgbd_server', anonymous=False)

        self._read_lock = threading.Lock()
        self._br = CvBridge()

        self._rgb_sub = rospy.Subscriber(rgb_image_topic,sensor_msgs.msg.Image, self.rgb_callback, queue_size=1)
        self._depth_sub = rospy.Subscriber(depth_image_topic, sensor_msgs.msg.Image, self.depth_callback, queue_size=1)
        self._pcl_sub = rospy.Subscriber(cloud_topic, sensor_msgs.msg.PointCloud2, self.pcl_callback, queue_size=1)
        self.camera_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self.camera_callback, queue_size=1)

        self._rgb_image = None
        self._depth_image = None
        self._pcl_data = None
        self._pcl_array = None
        self.camera_info = None

        self._rate = rospy.Rate(10) # 10hz
        self._still_running = True
        rospy.on_shutdown(self._shutdown)

    def _shutdown(self):
        self._still_running = False
        # self._tf_thread.join()

    def rgb_callback(self, data):
        if self._read_lock.locked():
            return
        self._rgb_image = self._br.imgmsg_to_cv2(data)

    def camera_callback(self,data):
        if self._read_lock.locked():
            return
        self.camera_info = data

    def depth_callback(self, data):
        if self._read_lock.locked():
            return
        self._depth_image = self._br.imgmsg_to_cv2(data)

    def pcl_callback(self, data):
        if self._read_lock.locked():
            return
        self._pcl_array = cloud_msg_to_np(data)
        self._pcl_data = pcl.PointCloud()
        self._pcl_data.from_array(self._pcl_array)

    def get_data(self):
        with self._read_lock:
            if not self.ready():
                return None
            # print 'get data'

            response = RGBDAggregate(
                rgb_image=self._rgb_image,
                depth_image=self._depth_image,
                camera_info=self.camera_info,
                point_cloud_array=self._pcl_array,
                point_cloud_pcl=self._pcl_data
            )
            return response

    def save_example(self, rgbd_example, save_location, index):
        if not os.path.isdir(save_location):
            os.makedirs(save_location)

        # np.savetxt(os.path.join(save_location, '{}_camera2world_tf.npy'.format(index)), rgbd_example.camera2world_tf_mat)
        cv2.imwrite(os.path.join(save_location,'{}_rgb.png'.format(index)), rgbd_example.rgb_image)
        rgbd_example.point_cloud_pcl.to_file(os.path.join(save_location, '{}_pcl.pcd'.format(index)))
        # cv2.imwrite(os.path.join(save_location, '{}_rgb.png'.format(index)), rgbd_example.rgb_image)
        cv2.imwrite(os.path.join(save_location, '{}_depth.png'.format(index)), rgbd_example.depth_image)
        with open(os.path.join(save_location, '{}_pkl.pkl'.format(index)), 'wb') as output:
            pickle.dump(rgbd_example.camera_info, output)

    def ready(self):
        return self._rgb_image is not None and self._depth_image is not None and self._pcl_array is not None and self._pcl_data is not None

    def __del__(self):
        self._shutdown()


def init_service(rgb_image_topic, depth_image_topic,camera_info_topic,cloud_topic):
    global _rgbd_data_collector
    _rgbd_data_collector = RGBDDataCollector(
        rgb_image_topic=rgb_image_topic,
        depth_image_topic=depth_image_topic,
        camera_info_topic=camera_info_topic,
        cloud_topic=cloud_topic
    )


def get_all_data():
    if not _rgbd_data_collector:
        raise ValueError("RGBDDataCollector has not been initialized. Did you call 'init_service'?")
    return _rgbd_data_collector.get_data()

def data_formatter(data_chunk):
    data_dict = {}
    meta = {}

    meta['factor_depth'] = np.array([[500]]).astype(np.uint16)
    meta['intrinsic_matrix'] = np.resize(np.asarray(data_chunk[2].K),(3,3))
    meta['projection_matrix'] = np.resize(np.asarray(data_chunk[2].P),(3,4))
    meta['rotation_translation_matrix'] = np.matmul(np.linalg.inv(meta['intrinsic_matrix']), meta['projection_matrix'])


    data_dict['rgb_image'] = data_chunk[0]
    data_dict['depth_image'] = data_chunk[1]
    data_dict['camera_info'] = data_chunk[2]
    data_dict['point_cloud_array'] = data_chunk[3]
    data_dict['meta_data'] = meta

    return data_dict


def save_rgbd_example(rgbd_example, save_location, index):
    # type: (RGBDAggregate, str, str) -> ()
    _rgbd_data_collector.save_example(rgbd_example, save_location, index)


def ready():
    return _rgbd_data_collector.ready()


def main():

    # kinect camera
    rgb_image_topic = "/camera/rgb/image_color"
    depth_image_topic = "/camera/depth/image_raw"
    camera_info_topic = "/camera/depth/camera_info"
    cloud_topic = '/camera/depth/points'

    # fetch robot
    rgb_image_topic = "/head_camera/rgb/image_raw"
    depth_image_topic = "/head_camera/depth/image_raw"
    cloud_topic = "/head_camera/depth/points"
    camera_info_topic = "/head_camera/depth/camera_info"

    save_location = 'examples'


    init_service(rgb_image_topic, depth_image_topic,camera_info_topic, cloud_topic)


    while not ready():
        time.sleep(0.5)

    rgbd_data = get_all_data()
    save_rgbd_example(rgbd_data, save_location, 1)


def data_getter():
    main()
    data_chunk = get_all_data()
    data_dict = data_formatter(data_chunk)

    # import ipdb
    # ipdb.set_trace()

    return data_dict

if __name__ == '__main__':
    main()
    print 'done'