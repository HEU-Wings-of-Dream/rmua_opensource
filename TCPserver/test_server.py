import socket
import string

#收到的字符串长度
BAG_SIZE = 7

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
address = ('192.168.43.213',60999)
serverSocket.bind(address)
serverSocket.listen()
print("waitting for connection")
clientSocket,clientInfo = serverSocket.accept()
print("connect complete!")
#print("receive " ,end=' ')
while True:
    data_recv = clientSocket.recv(1024).decode()#接受的字符
    if not data_recv:
        continue
    data = data_recv[len(data_recv) - BAG_SIZE:]
    print(data, end='\n')

clientSocket.close()
serverSocket.close()