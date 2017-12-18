
import numpy as np
import matplotlib.pyplot as plt


import os


if __name__ == '__main__':
    dir_name= '/home/steve/Code/Mini_IMU/Scripts/NewIU/13/'
    imu1_data = np.loadtxt(dir_name+'imu.txt',delimiter=',')
    imu2_data = np.loadtxt(dir_name+'imu2.txt',delimiter=',')

    uwb_file_name = ''
    for name in os.listdir(dir_name):
        if 'uwb' in name:
            uwb_file_name=name
    uwb_file = open(dir_name+uwb_file_name)

    uwb_file_list = uwb_file.readlines()

    start_time = float(uwb_file_name.split('_')[0])
    first_time = uwb_file_list[0].split('[')[1].split(']')[0]
    first_time = float(first_time)
    last_time = uwb_file_list[-2].split('[')[1].split(']')[0]
    last_time = float(last_time)
    end_time = last_time-first_time+start_time

    print(imu1_data[0,0]-start_time,imu2_data[0,0]-start_time)
    print(imu1_data[-1,0]-end_time,imu2_data[-1,0]-end_time)
    print('imu time:',imu1_data[-1,0]-imu1_data[0,0])
    print('imu2 time:',imu2_data[-1,0]-imu2_data[0,0])
    print('uwb time:',end_time-start_time)

