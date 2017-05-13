# -*- coding:utf-8 -*-
# Create by steve in 17-5-12 at 下午1:52

import serial
import sys


if __name__ == '__main__':

    dev_str = '/dev/ttyUSB1'
    dev_baud = 115200
    save_file_name = ''

    for tt in sys.argv:
        print("tt is :" ,tt)
        if 'dev' in tt:
            dev_str = tt
    print("begin")
    t = serial.Serial(dev_str,1382400)

    while 1:
        data = t.read(1)
        # print(data)
        # print(len(data))
        if data[0] == 0x55:
            chrtmp = t.read(10)
            # print("in")
            print(chrtmp)
            if chrtmp[0] == 0x50:
                print("chrtmp",chrtmp)








