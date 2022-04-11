#define USE_ROS 1

#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <thread>
#include <sys/shm.h>
#include <string>
#include <queue>

#if USE_ROS
#include <ros/ros.h>
#include <port_listener_core/Posemsg.h>
#include <ros/subscriber.h>
#include <geometry_msgs/Point.h>
#include <ros/publisher.h>
#include <ros/time.h>
#endif

#define BUFFER_SIZE 8

#define GOAL_IP "192.168.43.126"
#define GOAL_PORT "1953"

struct my_struct
{
    /* data */
    int x = 0;
    int y = 0;
} recv_buffer;

bool is_online = 0;
int sockfd;
int readLn = 0, my_count = 0;
std::string str1, str2, send_string;

int set_socket(int fd)
{
    int old_op = fcntl(fd, F_GETFL);
    int new_op = old_op | O_NONBLOCK;
    fcntl(fd, F_SETFL, new_op);
    return old_op;
}

int last_x = -999, last_y = -999;

bool need_reconnect = 0;

#ifdef USE_ROS
void position_listener_callback(const geometry_msgs:: Point :: ConstPtr& recv_msg)
{
    if ((int(recv_msg->x * 1000 / 20) == last_x) && (int(recv_msg->y * 1000 / 20) == last_y))
        return;

    // if (last_x == -999 && last_y == -999){
    //     last_x = int(recv_msg->x * 1000 / 20);
    //     last_y = int(recv_msg->y * 1000 / 20);
    //     return;
    // }
    // recv_buffer.x = recv_msg->x;
    // recv_buffer.y = -recv_msg->y;
    recv_buffer.x = int(recv_msg->x * 1000 / 20);
    recv_buffer.y = int(recv_msg->y * 1000 / 20);

    last_x = int(recv_msg->x * 1000 / 20);
    last_y = int(recv_msg->y * 1000 / 20);
    //recv_buffer.y = - recv_buffer.y;
    //printf("read call back\n");
    //printf("%d   %d\n", recv_buffer.x, recv_buffer.y);

    if (is_online == 0 || need_reconnect == 1) return;
    //printf("wait to send!\n");
    char send_buff[BUFFER_SIZE];
    if (recv_buffer.x < 100){
        if (recv_buffer.x < 10)
            str1 = "00" + std::to_string(recv_buffer.x);
        else
            str1 = '0' + std::to_string(recv_buffer.x);
    }
    else
        str1 = std::to_string(recv_buffer.x);

    if (recv_buffer.y < 100){
        if (recv_buffer.y < 10)
            str2 = "00" + std::to_string(recv_buffer.y);
        else
            str2 = '0' + std::to_string(recv_buffer.y);
    }
    else
        str2 = std::to_string(recv_buffer.y);

    send_string = str1 + ' ' + str2 + 's';

    for (int i = 0; i < send_string.length(); i++)
        send_buff[i] = send_string[i];
        
    send(sockfd, send_buff, sizeof(send_buff), 0);

    printf("Update message success!   Send:\n %s\n", send_buff);

    memset(send_buff, 0, sizeof(send_buff));

    

    return;
}
#endif

inline void TCP_init()
{
    sockfd=socket(AF_INET,SOCK_STREAM,0);
    //1、创建socket套接字

    if(sockfd<0){
        perror("create socket error");
        return;
    }
    printf("Create socket success!!!\n");

    struct sockaddr_in ser;
    memset(&ser, 0, sizeof(ser));
    ser.sin_family=AF_INET;
    ser.sin_addr.s_addr=inet_addr(GOAL_IP);
    ser.sin_port=htons(atoi(GOAL_PORT));
    
    //2、连接
    int new_op = set_socket(sockfd);

    //int flag = connect(sockfd,(struct sockaddr*)&ser,len);

    if(connect(sockfd,(struct sockaddr*)&ser,sizeof(ser))<0)
    {
        perror("connect error");
        return ;
    }
    else
        printf("connect success");

    printf("Connect success!!! Wait to listen.");
}

char char_recv_buffer[8];

std::string recv_buffer_string = "00000000";
std::queue <std::string> recv_buffer_queue;

int enemy_x = 0, enemy_y = 0;

void my_recv()
{
    char temp_x[4], temp_y[4];
    std::string temp_string;
    // printf("Ready to clean recv buffer!!\n");
    // while (readLn = recv(sockfd, char_recv_buffer, sizeof(recv_buffer), 0));
    // printf("Clean receive buffer!!\n");
    while (readLn = recv(sockfd, char_recv_buffer, sizeof(char_recv_buffer), 0))
    {
        if (readLn == -1){
            my_count++;
            //printf("read error\n");
            if (my_count == 900000000){
                is_online = 0;
                printf("Received nothing\n");
                my_count = 0;
                need_reconnect = 1;
                return;
            }
            continue;
        }
        my_count = 0;
        is_online = 1;

        printf("Received from 192.168.43.126:\n%s\n", char_recv_buffer);

        if (char_recv_buffer[7] != 'E'){
            temp_x[0] = temp_string[0]; temp_x[1] = temp_string[1]; temp_x[2] = temp_string[2];
            temp_y[0] = temp_string[4]; temp_y[1] = temp_string[5]; temp_y[2] = temp_string[6];
            enemy_x = atoi(temp_x); enemy_y = atoi(temp_y);

        }
            //continue;

        for (int i=0;i<=7;i++)
            recv_buffer_string[i] = char_recv_buffer[i];
        recv_buffer_queue.push(recv_buffer_string);
        //std::cout<<recv_buffer_string<<std::endl;
        
    }
}

void command_publisher(const ros::Publisher& publisher)
{
    std::string temp_string;
    ros::Rate my_rate(50);
    char temp_x[4], temp_y[4];
    geometry_msgs::Point send_buffer;
    int goalx, goaly;
    while (ros::ok())
    {
        if (recv_buffer_queue.empty() == 0){
            temp_string = recv_buffer_queue.front();
            recv_buffer_queue.pop();
            if (temp_string[7] == 'G'){
                temp_x[0] = temp_string[0]; temp_x[1] = temp_string[1]; temp_x[2] = temp_string[2];
                temp_y[0] = temp_string[4]; temp_y[1] = temp_string[5]; temp_y[2] = temp_string[6];
                goalx = atoi(temp_x); goaly = atoi(temp_y);
                send_buffer.x = goalx;
                send_buffer.y = goaly;
                send_buffer.z = enemy_x * 1000 + enemy_y;
                publisher.publish(send_buffer);
                printf("Published new goalx, goaly.  (%d,   %d,  %d)\n", goalx, goaly, send_buffer.z);
            }
        }
        ros::spinOnce();
        if (need_reconnect == 1)
            return;
        my_rate.sleep();
    }
}

struct sockaddr_in ser;

int main(int argc, char **argv)
{

#ifdef USE_ROS
    ros::init(argc, argv, "tcpcore");
    ros::NodeHandle communication_handle_obj;
    ros::Rate loop_rate(50);
#endif
    ros::Subscriber position_listener = communication_handle_obj.subscribe("/now_pos", 10, position_listener_callback);
    ros::Publisher TCP_command_publisher = communication_handle_obj.advertise<geometry_msgs::Point>("/TCP_command_listener", 10);

INIT:
    ///TCP_init();
    sockfd=socket(AF_INET,SOCK_STREAM,0);
    //1、创建socket套接字

    if(sockfd<0){
        perror("create socket error");
        return 0 ;
    }
    printf("Create socket success!!!\n");

    
    memset(&ser, 0, sizeof(ser));
    ser.sin_family=AF_INET;
    ser.sin_addr.s_addr=inet_addr(GOAL_IP);
    ser.sin_port=htons(atoi(GOAL_PORT));
    
    //2、连接
    //socklen_t len=sizeof(struct sockaddr_in);
    //int new_op = set_socket(sockfd);

    //int flag = connect(sockfd,(struct sockaddr*)&ser,len);
    

    if(connect(sockfd,(struct sockaddr*)&ser, sizeof(ser))<0)
    { 
        perror("connect error");
        close(sockfd);
        return 0;
    }
    else
        printf("connect success");

    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
          fprintf(stderr, "Get flags error:%s\n", strerror(errno));
          close(sockfd);
          return -1;
      }
      flags |= O_NONBLOCK;
      if (fcntl(sockfd, F_SETFL, flags) < 0) {
          fprintf(stderr, "Set flags error:%s\n", strerror(errno));
          close(sockfd);
          return -1;
      }

    printf("Update socket setting success!!!\n");

    printf("Connect success!!! Wait to listen.");


    std::thread recv_thread(&my_recv);
    recv_thread.detach();

    std::thread publish_thread(&command_publisher, (TCP_command_publisher));
    publish_thread.detach();
    
#if USE_ROS
    while (ros::ok())
    {
        if (need_reconnect == 1){
            close(sockfd);
            need_reconnect = 0;
            printf("Close socket success!!\n");
            goto INIT;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
#endif

#ifndef USE_ROS
    //3、发送数据
    while(1)
    {
            char buff[1024]={0};
            scanf("%s",buff);
            send(sockfd,buff,sizeof(buff),0);
            memset(buff,0x00,1024);

            int ret=recv(sockfd,buff,1023,0);
            if(ret<0){
                perror("recv error");
                continue;
            }
            printf("client %s[%d]say:%s",inet_ntoa(ser.sin_addr),ntohs(ser.sin_port),buff);
    }

    close(sockfd);
#endif

    return 0;
}