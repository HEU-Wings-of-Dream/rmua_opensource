#!/usr/bin/env python2
import socket
import string
import rospy
from std_msgs.msg import String

#收到的字符串长度
BAG_SIZE = 7

rospy.init_node('TCPlistener')
pub = rospy.Publisher('TCPlistener', String, queue_size=10)
rate = rospy.Rate(10)

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
address = ('192.168.43.213',60999)
serverSocket.bind(address)
serverSocket.listen()
print("waitting for connection")
clientSocket,clientInfo = serverSocket.accept()
print("connect complete!")

while True:
    data_recv = clientSocket.recv(1024).decode()#接受的字符
    if not data_recv:
        continue
    data = data_recv[len(data_recv) - BAG_SIZE:]
    pub.publish(data);
    print(data, end='\n')
    rospy.spin();
    rate.sleep()

clientSocket.close()
serverSocket.close()