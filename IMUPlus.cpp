//
// Created by steve on 17-5-17.
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
double now()
{
    auto tt = std::chrono::system_clock::now();
    auto t_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tt.time_since_epoch());

    double time_now(double(t_nanosec.count())*1e-9);
    return time_now;
}

unsigned char ucComNo[2] ={0,0};
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;
    if  ( tcgetattr( fd,&oldtio)  !=  0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero( &newtio, sizeof( newtio ) );
    newtio.c_cflag  |=  CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    switch( nBits )
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch( nEvent )
    {
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

    switch( nSpeed )
    {
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
            cfsetispeed(&newtio,B921600);
            cfsetospeed(&newtio,B921600);
            break;
        case 460800:
            cfsetispeed(&newtio,B460800);
            cfsetospeed(&newtio,B460800);
            break;

        default:
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
    }
    if( nStop == 1 )
    {
        newtio.c_cflag &=  ~CSTOPB;
    }
    else if ( nStop == 2 )
    {
        newtio.c_cflag |=  CSTOPB;
    }
    newtio.c_cc[VTIME]  = 0;
    newtio.c_cc[VMIN] = 0;
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    printf("set done!\n");
    return 0;
}
int main(int argc, char* argv[])
{
    std::string dev_str("/dev/ttyUSB0");
    std::string save_file("test"+std::to_string(now())+".txt");

    if(argc == 3)
    {
        dev_str=std::string(argv[1]);
        save_file = std::string(argv[2]);
    }


    CJY901 JY901;

    int fd = open(dev_str.c_str(), O_RDWR | O_NOCTTY);
    if(-1==fd)
    {
        std::cout << " can't open device"<< std::endl;
        return 0;
    }
    std::ofstream out_file(save_file);


    char chrBuffer[4000];
    unsigned short usLength=0,usCnt=0;

    set_opt(fd, 460800, 8, 'N', 1);


    float last_milisecond = (float)JY901.stcTime.usMiliSecond/1000;
    int error_times = 0;
    double start_time =now();
    out_file.precision(20);
    out_file << now() << std::endl;



}
