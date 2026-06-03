#最简单demo，不包含夹爪
#调试网络连接和基础控制
#直接运行即可

import serial,time,keyboard
import socket

def main():
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.settimeout(10)

    #网络连接
    # host = '192.168.0.3'
    host = '192.168.0.4'
    port = 30001
    mySocket.connect((host, port))

    #运行
    time.sleep(2)
    mySocket.send("movel(p[-0.077, -0.636, 0.341, 2.778, -0.994, 0.047], a=1.2, v=0.2)\n".encode())
    time.sleep(4.5)
    mySocket.send("movel(p[-0.091, -0.678, 0.214, 2.943, -1.057, 0.046], a=1.2, v=0.2)\n".encode())
    time.sleep(1.2)
    time.sleep(2)
    mySocket.send("movel(p[-0.077, -0.636, 0.341, 2.778, -0.994, 0.047], a=1.2, v=0.2)\n".encode())
    time.sleep(1.2)
    mySocket.send("movel(p[-0.553, -0.0188, 0.36397, 1.266, -2.572, -0.049], a=1.2, v=0.2)\n".encode())
    time.sleep(4.5)
    mySocket.send("movel(p[-0.524, -0.019, 0.211, 1.208, -2.883, -0.001], a=1.2, v=0.2)\n".encode())
    time.sleep(1.5)

    mySocket.close()

if __name__ == '__main__':
    main()

