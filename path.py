import os
import sys

def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

this_dir = os.path.dirname(__file__)

# Add lib to PYTHONPATH
lib_path = os.path.join(this_dir, 'lib')
add_path(lib_path)
lib_path = os.path.join(this_dir,'lib/kinect_fusion/build')
add_path(lib_path)



add_path('/home/weizhang/kinect_rgdb_collector_ws/devel/lib/python2.7/dist-packages')
add_path('/opt/ros/kinetic/lib/python2.7/dist-packages')
