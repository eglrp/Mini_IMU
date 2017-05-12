# -*- coding:utf-8 -*-
# Create by steve in 17-5-12 at 下午1:52

import serial


if __name__ == '__main__':
    print("begin")
    t = serial.Serial('/dev/ttyUSB1',1382400)

    while 1:
        data = t.read(1)
        # print(data)
        if data == 0x80:
            chrtmp = t.read(10)
            print("in")
            if chrtmp[0] == b'\x50':
                print("chrtmp",chrtmp)








