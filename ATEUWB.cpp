/** 
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
*/
//
// Created by steve on 17-11-20.
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
#include <regex>
#include <string>


#include <chrono>

#include "CharQueue.h"

/**
 * Check if usb device is ok
 * @param dev_name  '/dev/ttyUSB0'
 * @return
 */
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

/**
 *
 * @return unix time, unit s(double)
 */
double now() {
    auto tt = std::chrono::system_clock::now();
    auto t_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(tt.time_since_epoch());

    double time_now(double(t_nanosec.count()) * 1e-9);
    return time_now;
}

unsigned char ucComNo[2] = {0, 0};
bool StopExeFlag = false;

/**
 *  set parameters for serial port.
 * @param fd
 * @param nSpeed
 * @param nBits
 * @param nEvent
 * @param nStop
 * @return
 */
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

////// GLOBAL VALUE .....
//void ProcessAndSaveThread(int )



inline bool ProcessAndSaveThread(char *buf,
                                 int buf_size,
                                 const std::ofstream &out_file) {

    int tag_offset[4] = {0, 4, 7, 9};

    auto *dis_array = new double[10];
    std::string buf_str;
    buf_str.resize(buf_size);
    memcpy(const_cast<char *>(buf_str.c_str()), buf, buf_size);


//    std::cout << " insid//e function :\n "
//              << "|" << buf_str << "|\n";
//    std::cout.flush();


    // check one line
    std::regex l_reg(".{0,}(A1).*[\\r|\\n|\\r\\n]");

    const std::sregex_iterator end;
    for (std::sregex_iterator iter(buf_str.cbegin(),
                                   buf_str.cend(),
                                   l_reg);
         iter != end;
         ++iter) {
        std::cout << "iter :" << (*iter)[0] << std::endl;

        std::string current_line(iter->str());

        std::regex base_id("T\\d{2} M00 \\d{5}");


    }








//    std::cout << "|"
//              << buf[buf_size - 1]
//              << "|"
//              << std::endl;
    return true;
}


int main(int argc, char *argv[]) {
    std::string dev_str("/dev/ttyUSB0");
    std::string save_file("./testATEUWB.txt");

    if (argc == 3) {
        dev_str = std::string(argv[1]);
        save_file = std::string(argv[2]);
    }


    int fd = open(dev_str.c_str(), O_RDWR | O_NOCTTY);
    if (-1 == fd) {
        std::cout << " can't open device" << std::endl;
        return 0;
    }

    std::ofstream out_file(save_file);
    out_file.precision(13);


    char chrBuffer[4000];
    unsigned short usLength = 0, usCnt = 0;
    set_opt(fd, 115200, 8, 'N', 1);


    double last_time = 0.0;
    double now_time = 0.0;
    int readed_counter = 0;

    while (true) {

        memset(chrBuffer, 0, 4000);
        int len = read(fd, chrBuffer, 4000);
        now_time = now();
        readed_counter++;
        if (!checkUSB(dev_str)) {
            std::cout << dev_str << " disconnected" << std::endl;
            out_file.close();
            delete[] chrBuffer;
            return 0;
        }
        if (len > 0) {
            if (last_time > 10.0) {

                std::cout << now_time - last_time
                          << "  "
                          << readed_counter
                          << "\n";

            }

            last_time = now_time;
            readed_counter = 0;

            std::cout << "\n ----------------------\n"
                      << chrBuffer
                      << std::endl;


            ProcessAndSaveThread(chrBuffer, len, out_file);

            std::cout << "\n ============" << std::endl;

        }

    }

}
