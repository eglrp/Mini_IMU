import os



if __name__ == '__main__':
    base_dir = './II/'
    a = 10
    for dir in os.listdir(base_dir):
        if float(dir)>=a:
            a = int(dir)+1

    sub_dir_name = base_dir+str(a)

    os.mkdir(sub_dir_name)

    IMU_dev = '/dev/ttyUSB0'
    IMU_file = sub_dir_name+'/imu.txt'

    IMU2_dev = '/dev/ttyUSB1'
    IMU2_file = sub_dir_name+'/imu2.txt'

    os.system('../cmake-build-debug/IMUPlus {0} {1} & ../cmake-build-debug/IMUPlus {2} {3}'.format(
        IMU_dev,
        IMU_file,
        IMU2_dev,
        IMU2_file
    ))