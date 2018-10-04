#coding=utf-8
from socket import *
import time
import matplotlib.pyplot as plt
import csv
import math

# 1. 创建udp套接字
udp_socket = socket(AF_INET, SOCK_DGRAM)
i=0
# 2. 准备接收方的地址
dest_addr = ('192.168.1.199', 5000)

# 3. 获取数据到列表中
SendDataList=[0xAB,0xCD,0x01,0X02,0X03,0X04,0X05,0X06,0X07] ## SendDataList=[0xAB,0xCD,0xBA] , the settings will trigger STM32 W5500 transmit data back to Computer termination.
# When STM32 has received 0xAB for 1st byte and 0xCD for 2nd byte , 
# STM32 W5500 would send the whole received information to the other termination. 
# By Sun Libo Tel and WeChat : 15889672958

SendContents=bytes(SendDataList)


# 4. 发送数据到指定的设备上
udp_socket.sendto(SendContents, dest_addr)

# 5. 等待接收对方发送的数据 
# 如果没有收到数据则会阻塞等待 直到收到数据
recv_data = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
# 6. 显示对方发送的数据
# 接收到的数据recv_data是一个元组
# 第1个元素是对方发送的数据
# 第2个元素是对方的IP地址和端口

ReadBackData=[]        
for i in range(len(recv_data[0])):
    ReadBackData.append(recv_data[0][i])
    print(ReadBackData[i])

print(recv_data[1])##打印对方的IP地址和端口

# 7. 关闭套接字
udp_socket.close()
