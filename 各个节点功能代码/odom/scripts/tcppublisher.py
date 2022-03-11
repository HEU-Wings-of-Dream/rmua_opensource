#!/usr/bin/env python3.8
import rospy
import sys
import socket
from odom.msg import Posemsg

def poseCallback(msg):
    rospy.loginfo("tcp publish pose: x:%f, y:%f, angle:%f", msg.x, msg.y, msg.angle)
    #posestring = str(msg.x) + ' ' + str(msg.y) + ' ' + str(msg.angle)
    posestring = str(123.111)
    try:
    	sock.send(posestring.encode())#发送字符
    except BrokenPipeError:
    	#sock.close()
    	#global sock
    	#sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    	address = ('192.168.43.213',60999)
    	sock.connect(address)

def wifipublisher():
	# ROS节点初始化
    rospy.init_node('wifipublisher', anonymous=True)

	# 创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
    rospy.Subscriber("send_to_server_now_pose", Posemsg, poseCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
	#print(sys.version)
	#help()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(None)
    address = ('192.168.43.213',60999)
    print('trying to connect service')
    flag = 0
    while (flag == 0):
    	try:
    		sock.connect(address)
    	except ConnectionRefusedError:
    		print("connect refused!,retrying")
    		flag = 0
    	else:
    		print("connect complete!!")
    		flag = 1
    #print('witing service for answer')
    print('connected from:{}'.format(address))
    wifipublisher()
