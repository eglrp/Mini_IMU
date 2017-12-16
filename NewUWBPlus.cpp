//
// Created by steve on 17-5-31.
//


#include <iostream>
#include     <stdio.h>      /*标准输入输出定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*Unix 标准函数定义*/
#include     <sys/types.h>
#include     <sys/stat.h>

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include "time.h"
#include "stdio.h"
#include "JY901.h"

//#include <boost/asio.hpp>
//#include <boost/bind.hpp>

#include <fstream>
#include <cmath>

#include <mutex>
#include <thread>


#include <chrono>

#include "CharQueue.h"

bool checkUSB(std::string dev_name) {
    FILE *stream;
    char out_res[1000];
    stream = popen(std::string("ls " + dev_name).c_str(), "r");
    memset(out_res, '\0', sizeof(out_res));
    fread(out_res, sizeof(char), sizeof(out_res), stream);
    pclose(stream);
//    std::cout << std::string(out_res).size() << std::endl;
    if (std::string(out_res).size() == 0) {
        return false;
    } else {
        return true;
    }
}

double now() {
    auto tt = std::chrono::system_clock::now();
    auto t_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tt.time_since_epoch());

    double time_now(double(t_nanosec.count()) * 1e-9);
    return time_now;
}

int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop) {
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch (nEvent) {
        case 'O':                     //奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':                     //偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':                    //无校验
            newtio.c_cflag &= ~PARENB;
            break;
    }

    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 921600:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;

        default:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
    }
    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;
    }
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd, TCIFLUSH);
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}

int main(int argc, char *argv[])
{
    std::string dev_str("/dev/ttyUSB1");
    std::string save_file_dir("./");

    if(argc == 3)
    {
        dev_str = std::string(argv[1]);
        save_file_dir = std::string(argv[2]);
    }

    int fd = open(dev_str.c_str(),O_RDWR|O_NOCTTY);
    if ( -1 == fd)
    {
        std::cout << " There is a Error when try to open the device :" << dev_str<< std::endl;
        return 0;
    }

    set_opt(fd,916200,8,'N',1);

    int buff_size(22222);
    char buff[buff_size];
    memset(buff,'\0',buff_size);


    while(true)
    {
        int data_size = read(fd,buff,buff_size);
        if(data_size<buff_size-1)
        {
            break;
        }

    }

    double start_time = now();

//    int out_file = open(std::string(save_file_dir+std::to_string(start_time)+"_uwbdata.txt").c_str(),
//                        O_WRONLY|O_APPEND);
//
//    close(out_file);
    std::ofstream out_file(std::string(save_file_dir
                                       +std::to_string(start_time)+
                                               "_uwbdata.txt"));


    std::cout << "start save data" << std::endl;
    int uwb_counter_times = 0;

    while(true)
    {

        memset(buff,'\0',buff_size);

        int data_size = read(fd,buff,buff_size);

        if(data_size>0)
        {
//
//            int out_file = open(std::string(save_file_dir+std::to_string(start_time)+"_uwbdata.txt").c_str(),
//                                O_WRONLY|O_APPEND);
//            write(out_file,buff,buff_size);
//
//            write(0,buff,buff_size);
//            close(out_file);
            out_file<< buff;
//            std::cout << buff;



        }else{
//            usleep(10);
//            std::cout.flush();
            if(uwb_counter_times<10)
            {
                uwb_counter_times++;
            }else{

                out_file.flush();
                uwb_counter_times=0;
            }
            if(!checkUSB(dev_str))
            {

                out_file.flush();
//                close(out_file);
                out_file.close();
                close(fd);
                return 0;
            }
        }

    }




}