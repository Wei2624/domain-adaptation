import socket
# from pose_estimation import ransac
import sys

HOST = '192.168.1.2'
PORT = 65432

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(10)
conn, addr = s.accept()
while True:
	conn.sendall(b'Hello, world')

