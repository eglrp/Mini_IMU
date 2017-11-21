# Created by steve on 17-11-21 下午3:36
'''
                   _ooOoo_ 
                  o8888888o 
                  88" . "88 
                  (| -_- |) 
                  O\  =  /O 
               ____/`---'\____ 
             .'  \\|     |//  `. 
            /  \\|||  :  |||//  \ 
           /  _||||| -:- |||||-  \ 
           |   | \\\  -  /// |   | 
           | \_|  ''\---/''  |   | 
           \  .-\__  `-`  ___/-. / 
         ___`. .'  /--.--\  `. . __ 
      ."" '<  `.___\_<|>_/___.'  >'"". 
     | | :  `- \`.;`\ _ /`;.`/ - ` : | | 
     \  \ `-.   \_ __\ /__ _/   .-` /  / 
======`-.____`-.___\_____/___.-`____.-'====== 
                   `=---=' 
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ 
         佛祖保佑       永无BUG 
'''

import os



if __name__ == '__main__':
    base_dir = '/home/steve/Code/Mini_IMU/Scripts/IIU/'
    a = 10
    for dir in os.listdir(base_dir):
        if float(dir)>=a:
            a = int(dir)+1

    sub_dir_name = base_dir+str(a)

    os.mkdir(sub_dir_name)


    UWB_dev = '/dev/ttyUSB0'
    UWB_file = sub_dir_name+'/uwb.txt'

    IMU_dev = '/dev/ttyUSB1'
    IMU_file = sub_dir_name+'/imu.txt'

    IMU2_dev = '/dev/ttyUSB2'
    IMU2_file = sub_dir_name+'/imu2.txt'



    os.system('../cmake-build-debug/ATEUWB {0} {1} & ../cmake-build-debug/IMUPlus {2} {3} & ../cmake-build-debug/IMUPlus {4} {5}'.format(
        UWB_dev,
        UWB_file,
        IMU_dev,
        IMU_file,
        IMU2_dev,
        IMU2_file
    ))