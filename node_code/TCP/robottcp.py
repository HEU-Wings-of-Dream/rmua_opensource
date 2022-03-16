import socket
import threading
 
 
def send_msg(client):
    while True:
        msg = input('请输入您要发送的消息\n若要退出请输入"0":\n').strip()
        if msg == '0':
            break
        elif len(msg) == 0:
            print('空消息无法发送，请重试~！')
            continue
 
        msg = msg.encode('gbk')
        try:
            client.send(msg)
            print('发送成功~！')
        except Exception as err:
            print(err)
 
 
def receive_msg(client):
    while True:
        try:
            data = client.recv(1024).decode('gbk')
            if len(data) == 0:
                print('服务端主动断开...')
                break
                # continue
            else:
                print('{}'.format(data))
        except Exception as err:
            print(err)
            break
 
 
if __name__ == '__main__':
    # 创建客户端socket
    tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 客户端socket连接到 服务端
    tcp_client.connect(('127.0.0.1', 8010))
 
    recv_theard = threading.Thread(target=receive_msg, args=(tcp_client,))
 
    recv_theard.setDaemon(True)
 
    recv_theard.start()
 
    # 接受消息用 子线程, 子线程recv() 是堵塞的
    # 发送消息用 主线程, 主线程接受用户指令input() 是堵塞的
    send_msg(tcp_client)
 
    tcp_client.close()