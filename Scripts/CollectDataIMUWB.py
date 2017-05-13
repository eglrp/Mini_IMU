# -*- coding:utf-8 -*-
# Create by steve in 17-5-12 at 下午8:43


import os
import Scripts.UwbReader



if __name__ == '__main__':
    base_dir = '/home/steve/Data/IMUWB/'
    a = 10
    for dir in os.listdir(base_dir):
        if float(dir)>=a:
            a = int(dir)+1

    sub_dir_name = base_dir+str(a)

    os.mkdir(sub_dir_name)

    IMU_dev = '/dev/ttyUSB1'
    IMU_file = sub_dir_name+'/imu.txt'

    UWB_dev = '/dev/ttyUSB2'
    UWB_dir = sub_dir_name+'/'

    os.system('../cmake-build-debug/Mini_IMU {0} {1} & python ./UwbReader.py {2} {3}'.format(
        IMU_dev,
        IMU_file,
        UWB_dev,
        UWB_dir
    ))












