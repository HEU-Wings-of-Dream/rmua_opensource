#pragma once
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <string>
#include "winsock2.h"
#include <cstdlib>
#include <WS2tcpip.h>
#pragma comment(lib,"ws2_32.lib")//引用库文件

#define OPEN_COMMUNICATION 0
#define GOAL_IP "127.0.0.1"
#define LISTEN_PORT 1953

WSADATA wsaData;
char buff[1024];
SOCKADDR_IN addrSrv;
SOCKET sockClient, sockPublisher;
SOCKET sockSrv = socket(AF_INET, SOCK_STREAM, 0);

void listen_init()
{
	//加载套接字
	memset(buff, 0, sizeof(buff));

	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(9000);//端口号
	//IP地址
	char s[10] = GOAL_IP;
	//cout<<inet_pton(AF_INET,(const char*)&s, &addrSrv.sin_addr.S_un.S_addr)<<endl;
	InetPton(AF_INET, TEXT(GOAL_IP), &addrSrv.sin_addr.S_un.S_addr);
	//addrSrv.sin_addr.S_un.S_addr = inet_addr(GOAL_IP);
	//cout << addrSrv.sin_addr.S_un.S_addr;
	//创建套接字
	sockClient = socket(AF_INET, SOCK_STREAM, 0);
	if (SOCKET_ERROR == sockClient) {
		printf("Socket() error:%d", WSAGetLastError());
		return;
	}
}

void listen_connect()
{
	//向服务器发出连接请求
	if (connect(sockClient, (struct  sockaddr*)&addrSrv, sizeof(addrSrv)) == INVALID_SOCKET) {
		printf("Connect Failed:%d", WSAGetLastError());
		return;
	}
	else
	{
		////接收数据
		//recv(sockClient, buff, sizeof(buff), 0);
		//printf("%s\n", buff);
		printf("Connect Success!");
	}
}

void publish_init()
{
    SOCKADDR_IN addrSrv;
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(LISTEN_PORT); //1024以上的端口号
	/**
	 * INADDR_ANY就是指定地址为0.0.0.0的地址，这个地址事实上表示不确定地址，或“所有地址”、“任意地址”。 一般来说，在各个系统中均定义成为0值。
	 */
	addrSrv.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
	int retVal = bind(sockSrv, (LPSOCKADDR)&addrSrv, sizeof(SOCKADDR_IN));

	if (retVal == SOCKET_ERROR) {
		printf("连接失败:%d\n", WSAGetLastError());
		return;
	}

	if (listen(sockSrv, 10) == SOCKET_ERROR) {
		printf("监听失败:%d", WSAGetLastError());
		return;
	}

	SOCKADDR_IN addrClient;
	int len = sizeof(SOCKADDR);

    while (1)
	{
		//等待客户请求到来
		sockPublisher = accept(sockSrv, (SOCKADDR *)&addrClient, &len);
		if (sockPublisher == SOCKET_ERROR) {
			printf("等待请求失败:%d", WSAGetLastError());
			break;
		}

		printf("客户端的IP是:[%s]\n", inet_ntoa(addrClient.sin_addr));

		//发送数据
		// char sendbuf[] = "你好，我是服务端，咱们一起聊天吧";
		// int iSend = send(sockPublisher, sendbuf, sizeof(sendbuf), 0);
		// if (iSend == SOCKET_ERROR) {
		// 	printf("发送失败");
		// 	break;
		// }
		// HANDLE hThread = CreateThread(NULL, 0, Fun, NULL, 0, NULL);
		// CloseHandle(hThread);

	}
}

std::string my_receive()
{
	//char buffs[];
	//接收数据
	recv(sockClient, buff, sizeof(buff), 0);
	std::string buff_recv(buff);
	printf("%s\n", buff);
	return buff_recv;
}

void close_connect()
{
	//关闭套接字
	closesocket(sockClient);
	WSACleanup();//释放初始化Ws2_32.dll所分配的资源。
	system("pause");//让屏幕暂留
	return;
}

std::string format_function(std::string str)
{
    
}

void publish(std::string str)
{
    char sendbuf[100];
    strncpy(sendbuf, str.c_str(), str.length()+1);///s.c_str() 字符强制转化为字符串函数
    int iSend = send(sockPublisher, sendbuf, sizeof(sendbuf), 0);
    if (iSend == SOCKET_ERROR) {
		printf("send failed!! \n");
        return;
	}
}

int main()
{
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		printf("init Winsock failed");
		return;
	}

    listen_init();
    listen_connect();
    publish_init();

    std::string str; 
    while (true){
        str = my_receive();
        publish(format_function(str));
    }
}