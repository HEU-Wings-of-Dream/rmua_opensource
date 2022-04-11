#!/usr/bin/env python3
#coding=utf-8
import rospy
import sys
import socket
import threading
from port_listener_core.msg import Posemsg
from geometry_msgs.msg import Point32

# 创建客户端socket
tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 客户端socket连接到 服务端
tcp_client.connect(('192.168.43.213', 2022))

print ("socket init success!!!!")

def poseCallback(msg):
    rospy.loginfo("tcp publish pose: x:%f, y:%f, angle:%f", msg.x, msg.y, msg.angle)

    send_str1 = ''
    send_str2 = ''
    
    if (msg.x < 100):
        send_str1 = '0' + str(int(msg.x))
    else:
        send_str1 = str(int(msg.x))
    
    if (msg.y < 100):
        send_str2 = '0' + str(int(msg.y))
    else:
        send_str2 = str(int(msg.y))

    tcp_client.send(send_str1 + ' ' + send_str2)

    #print ("send msg success!!!")

    


# def send_msg(client):
#     while True:
#         msg = input('请输入您要发送的消息\n若要退出请输入"0":\n').strip()
#         if msg == '0':
#             break
#         elif len(msg) == 0:
#             print('空消息无法发送，请重试~！')
#             continue
 
#         msg = msg.encode('gbk')
#         try:
#             client.send(msg)
#             print('发送成功~！')
#         except Exception as err:
#             print(err)
 
 
def receive_msg(client):
    while True:
        try:
            data = client.recv(1024).decode('utf-8')
            if len(data) == 0:
                print('服务端主动断开...')
                break
                # continue
            else:
                print('{}'.format(data))
                my_received_msg = str(data)
                #find the ' '
                rest_position = my_received_msg.find(' ')
                #find the end of a bag
                end_position = my_received_msg.find('e')
                received_x = my_received_msg[0: rest_position]
                received_y = my_received_msg[rest_position+1 : end_position]
                int_x = int(received_x)
                int_y = int(received_y)
                
                send_buffer = Point32(int_x, int_y, 0)

                command_publisher.publish(send_buffer)
                print("send command x = %s, y = %s" % ( send_buffer.x, send_buffer.y))


        except Exception as err:
            print(err)
            break
 
 
def spin_function():
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('tcp_node', anonymous=True)

    my_rate = rospy.Rate(100)
    command_publisher = rospy.Publisher("/TCP_command_listener", Point32, queue_size = 10)
    #rospy.Subscriber("send_to_server_now_pose", Posemsg, poseCallback)
    
    recv_theard = threading.Thread(target=receive_msg, args=(tcp_client,))
 
    recv_theard.setDaemon(True)
 
    recv_theard.start()

    print("ready to listen to odom")

    while not rospy.is_shutdown():
        #print("111")
        msg = rospy.wait_for_message("/send_to_server_now_pose", Posemsg, None)
        #print("222")
        #rospy.loginfo("tcp publish pose: x:%f, y:%f, angle:%f", msg.x, msg.y, msg.angle)

        send_str1 = ''
        send_str2 = ''
        
        send_str1 = str(int(msg.x))
        
        send_str2 = str(int(msg.y))
        
        data_send = send_str1 + ' ' + send_str2
        
        tcp_client.send(data_send.encode("utf-8"))
        
        #print(send_str1 + ' ' + send_str2)

        #print ("send msg success!!!")

    

    #rospy.spin()
    # while True:
    #     my_rate.sleep()
    tcp_client.close()
