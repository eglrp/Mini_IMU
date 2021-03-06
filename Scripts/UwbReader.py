# -*- coding:utf-8 -*-
# carete by steve at  2017 / 05 / 07　14:35

import serial
import time

import sys


class UwbReader:
    def __init__(self, dev_name, save_file_name):
        self.save_source_file = open(save_file_name, 'wb')
        t = serial.Serial(dev_name,115200);
        while(t.is_open()):
            data = t.readline()
            print(data)
            save_source_file.write(data)

        t.close()
        save_source_file.close()




if __name__ == '__main__':
    dev_name = '/dev/ttyUSB1'
    save_dir = './'


    for tt in sys.argv:
        if 'dev' in tt:
            dev_name = tt
        elif 'IMU' in tt:
            save_dir=tt


    start_time = time.time()
    print(start_time)
    # save_source_file = open(save_dir + str(start_time) + "_uwbdata.txt", 'wb')
    is_first_time=True

    # print(time.strftime("%Y-%m-%d", start_time))
    t = serial.Serial(dev_name, 115200)
    not_use_data = t.read_all()
    while (t.isOpen()):
        if is_first_time:
            save_source_file = open(save_dir + str(start_time) + "_uwbdata.txt", 'wb')
            is_first_time = False
        data = t.readline()
        print(data)
        save_source_file.write((data))

    t.close()
    save_source_file.close()
