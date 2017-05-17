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

#include "CharQueue.h"

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

void ReadThread(int fd,CharQueue<char> & cq){
//   std::thread::
    usleep(100000);
    std::cout << "start read" << std::endl;
    while(true)
    {
        char  buff[4096];
        int length = read(fd,buff,4000);
        if(length>0)
        {
            cq.AddBuf(buff,length);
        }
        usleep(100);
    }
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

    CharQueue<char> cq(10000);

    std::thread t(ReadThread,fd,std::ref(cq));
    t.detach();

    sleep(1);

    while(1)
    {
        char  t[2];
        cq.ReadBuf(t,2);
//        std::cout << "size " << cq.getSize() << std::endl;
        if((t[0])==0x55){
            std::cout << "break" << std::endl;
            break;
        }else{
//            std::cout << t[0] << std::endl;
//            printf("is : %02X\n",t[0]);
            cq.DeletBuf(1);
        }
        usleep(1000);
    }



    while(1)
    {

//        usLength = CollectUARTData(ulComNo,chrBuffer);
//        usLength = read(fd, chrBuffer, 4000);
//        boost::system::error_code err;
//        usLength = sp.read_some(boost::asio::buffer(chrBuffer,1000),err);

//        if(cq.ReadBuf(chrBuffer,1))
//        {
//            if
//        }
        if(cq.ReadBuf(chrBuffer,5))
        {
            cq.DeletBuf(5);
        }else{
//            std::cout << "wait ?" << std::endl;
            usleep(1000);
            continue;
        }



        if (usLength>0)
        {
//            auto t= std::thread((JY901.CopeSerialData),chrBuffer,usLength);
//            t.detach();
            JY901.CopeSerialData(chrBuffer,usLength);
        }
//        Sleep(100);
//        usleep(50);

        if (usCnt++>=0&&JY901.getisend())//|| last_milisecond!=(float)JY901.stcTime.usMiliSecond/1000)
        {
            usCnt=0;

            if(std::fabs(std::fabs((float)JY901.stcTime.usMiliSecond/1000-last_milisecond)-0.005)>0.0001&&
               std::fabs(std::fabs((float)JY901.stcTime.usMiliSecond/1000-last_milisecond)-0.995)>0.0001    )
            {
//               std::cout <<  (float)JY901.stcTime.usMiliSecond/1000-last_milisecond ;//<< std::endl;
                error_times++;
                if(error_times%1==0)
                {
                    std::cout << error_times <<"average time :" << (now()-start_time)/double(error_times)<< std::endl;
                }

            }
            printf("Time:20%d-%d-%d %d:%d:%.3f",(short)JY901.stcTime.ucYear,(short)JY901.stcTime.ucMonth,
                   (short)JY901.stcTime.ucDay,(short)JY901.stcTime.ucHour,(short)JY901.stcTime.ucMinute,(float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);


            printf("Acc:%.3f %.3f %.3f",(float)JY901.stcAcc.a[0]/32768*16,(float)JY901.stcAcc.a[1]/32768*16,(float)JY901.stcAcc.a[2]/32768*16);

            printf("Gyro:%.3f %.3f %.3f",(float)JY901.stcGyro.w[0]/32768*2000,(float)JY901.stcGyro.w[1]/32768*2000,(float)JY901.stcGyro.w[2]/32768*2000);

            printf("Angle:%.3f %.3f %.3f\r\n",(float)JY901.stcAngle.Angle[0]/32768*180,(float)JY901.stcAngle.Angle[1]/32768*180,(float)JY901.stcAngle.Angle[2]/32768*180);

            printf("Mag:%d %d %d",JY901.stcMag.h[0],JY901.stcMag.h[1],JY901.stcMag.h[2]);

            printf("Pressure:%lx Height%.2f",JY901.stcPress.lPressure,(float)JY901.stcPress.lAltitude/100);

            printf("DStatus:%d %d %d %d\r\n",JY901.stcDStatus.sDStatus[0],JY901.stcDStatus.sDStatus[1],JY901.stcDStatus.sDStatus[2],JY901.stcDStatus.sDStatus[3]);

            printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",JY901.stcLonLat.lLon/10000000,(double)(JY901.stcLonLat.lLon % 10000000)/1e5,JY901.stcLonLat.lLat/10000000,(double)(JY901.stcLonLat.lLat % 10000000)/1e5);

            printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n\r\n",(float)JY901.stcGPSV.sGPSHeight/10,(float)JY901.stcGPSV.sGPSYaw/10,(float)JY901.stcGPSV.lGPSVelocity/1000);

            out_file << (short) JY901.stcTime.ucYear << "-" << (short) JY901.stcTime.ucMonth << "-" <<
                     (short) JY901.stcTime.ucDay << "-" <<
                     (short) JY901.stcTime.ucHour << ":" <<
                     (short) JY901.stcTime.ucMinute << ":"
                     << (float) JY901.stcTime.ucSecond <<"."<< (float) JY901.stcTime.usMiliSecond <<" " <<
                     //            out_file.precision(30);
                     //            out_file<<now()<<" "<<
                     (float) JY901.stcAcc.a[0] / 32768 * 16 << " " << (float) JY901.stcAcc.a[1] / 32768 * 16<<" "<<
                     (float) JY901.stcAcc.a[2] / 32768 * 16 << " " <<
                     (float) JY901.stcGyro.w[0] / 32768 * 2000<<" " <<  (float) JY901.stcGyro.w[1] / 32768 * 2000 << " " <<
                     (float) JY901.stcGyro.w[2] / 32768 * 2000 << " " <<
                     JY901.stcMag.h[0] << " " <<  JY901.stcMag.h[1] << " " << JY901.stcMag.h[2];
            out_file << std::endl;
            last_milisecond=(float)JY901.stcTime.usMiliSecond/1000;

        }

    }

}
