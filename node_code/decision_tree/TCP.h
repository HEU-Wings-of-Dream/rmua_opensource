#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <winsock2.h>
#include <string>
#include <queue>

#define GOAL_IP_ADDRESS "192.168.43.126"
#define GOAL_PORT 1953

WSADATA wsaData;
SOCKET hSocket;
SOCKADDR_IN servAddr;
char message[8];

std::queue <std::string> msg_recv_queue;
std::string msg_recv_temp;

std::queue <std::string> need_to_send_queue;
std::string need_to_send_msg;

char send_buffer[8];

bool TCP_init()
{
    /*WSADATA wsaData;
    SOCKET hSocket;
    SOCKADDR_IN servAddr;*/

    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        printf("WSAStartup() error!");
        return 0;
    }

    hSocket = socket(PF_INET, SOCK_STREAM, 0);
    if (hSocket == INVALID_SOCKET) {
        printf("hSocket() error");
        return 0;
    }

    memset(&servAddr, 0, sizeof(servAddr));
    servAddr.sin_family = AF_INET;
    servAddr.sin_addr.s_addr = inet_addr(GOAL_IP_ADDRESS);
    servAddr.sin_port = htons(GOAL_PORT);//atoi()

    if (connect(hSocket, (SOCKADDR*)&servAddr, sizeof(servAddr)) == SOCKET_ERROR) {
        printf("connect() error!");
        return 0;
    }

    return 1;
}

void my_recv(void)
{
    int readLen = 0;
    int my_count = 0;
    bool is_TCPserver_Online = 0;
    while (readLen = recv(hSocket, message, sizeof(message), 0))
    {
        //读取失败计数并提示
        if (readLen == -1) {
            is_TCPserver_Online = 0;
            my_count++;
            if (my_count == 2500000) {
                printf("Received Nothing\n");
                my_count = 0;
            }
            continue;
        }
        is_TCPserver_Online = 1;

        //如果需要发送的话
        if ((is_TCPserver_Online == 1) && (need_to_send_queue.empty() == 0)) {
            need_to_send_msg = need_to_send_queue.front();
            need_to_send_queue.pop();
            for (int i = 0; i <= 6; i++) {
                send_buffer[i] = need_to_send_msg[i];
            }
            send_buffer[7] = 'G';   //Ready to send message, end with 'G', which means "goal"
            send(hSocket, send_buffer, 8, 0);
            printf("send msg success. msg is %s\n", send_buffer);
        }

        //将收到的消息放入消息队列
        //if (message[7] == 'p') {  //Received enemy position, end with 'p', which means "position"
        msg_recv_temp = message;
        msg_recv_queue.push(msg_recv_temp);
        //printf("Received enemy locate, ready to search attack position for (%s)\n", message);

        //   std::cout << "Received self locate  " << msg_recv_temp << std::endl;

        memset(message, 0, sizeof(message));
    }
}