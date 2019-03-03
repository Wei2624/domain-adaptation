#!/usr/bin/env python


# import socket
# # from pose_estimation import ransac
# import sys
#
# HOST = '192.168.1.3'
# PORT = 65432
#
# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind((HOST, PORT))
# s.listen(10)
# conn, addr = s.accept()
# while True:
# 	conn.sendall(b'Hello, world')



#!/usr/bin/env python

import socket
from sys import getsizeof
import sys


# TCP_IP = '192.168.50.208'
TCP_IP = '0.0.0.0'
TCP_PORT = 65432
BUFFER_SIZE = 1024  # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

data_string = '1000'
for i in xrange(11):
    data_string += data_string

data_string = '2' + data_string
# print getsizeof(data_string)
# sys.exit()

conn, addr = s.accept()
print 'Connection address:', addr

while 1:
    # data = conn.recv(BUFFER_SIZE)
    print 'sending'
    # if not data: break
    # print "received data:", data
    # conn.send("hello world returned")  # echo
    conn.send(data_string)  # echo
conn.close()


# import rospy
# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import MultiArrayLayout
# import numpy as np
#
# import datetime, threading, time
#
# next_call = time.time()
#
# test_data = []
#
# def talker():
#     global next_call
#     pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     count = 0
#     # next_call = next_call+1
#     # threading.Timer( next_call - time.time(), talker ).start()
#     # hello_str = "hello world %s" % rospy.get_time()
#     # # data = np.zeros((10,3)).astype(np.float32)
#     # # data = [count] + [10]*1000000
#     # # data = [1,2,3,4]
#     # # data_pack = [MultiArrayLayout,data]
#     # print test_data
#     # data_pack = Float32MultiArray(data=test_data)
#     #
#     # rospy.loginfo(hello_str)
#     # pub.publish(data_pack)
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         data = np.zeros((10,3)).astype(np.float32)
#         data = [count] + [10.0]*2000000
#         # data = [1,2,3,4]
#         data_pack = [MultiArrayLayout,data]
#         data_pack = Float32MultiArray(data=data_pack)
#
#         rospy.loginfo(hello_str)
#         pub.publish(data_pack)
#         rate.sleep()
#         count += 1
#
#
#
# if __name__ == '__main__':
#     try:
#         test_data = [1,2,3,4]
#         talker()
#         #
#         # test_data = [2,2,3,4]
#         # talker()
#         print 'done'
#     except rospy.ROSInterruptException:
#         pass
