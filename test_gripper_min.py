#夹爪控制

import serial,time

def main():
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

    motor_open_list = (0x02,0x00,0x20,0x2f,0x00,0,0xa4) #机械爪松开(具体解释见机械爪用户手册)
    motor_close_list = (0x02,0x01,0x20,0x2f,0x00,0,0xa4)    #机械爪闭合，45字节是角度

    print('开始执行程序！')

    ser.write(motor_close_list)     #抓取
    time.sleep(5)
    ser.write(motor_open_list)  #放开

    ser.close()

if __name__ == '__main__':
    main()

