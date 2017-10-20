//
// Created by steve on 17-5-17.
//

#ifndef MINI_IMU_CHARQUEUE_H
#define MINI_IMU_CHARQUEUE_H

#include "stdlib.h"

#include "iostream"

#include <deque>

#include <thread>
#include <mutex>


/**
 *
 *
 */
template<typename T>
class CharQueue {

public:
    CharQueue(int size = 10000) {
        try {

//            data_buf_.resize(size);
            data_mutex_.lock();
            data_mutex_.unlock();
        } catch (std::exception &e) {
            std::cout << e.what() << std::endl;
        }
    }


    /**
     * Add buf to tail of the queue.
     * @param buff data point
     * @param len length of data
     * @return true :
     */
    bool AddBuf(T buff[], int len) {
        if (len <= 0) {
            return true;
        } else {
            data_mutex_.lock();
            memcpy(&data_buf_[data_length_],buff,len);
            data_length_+=len;
            data_mutex_.unlock();
            return true;
        }
    }

    /**
     * Read first several data from queue.
     * @param buff  point for buff
     * @param len length
     * @return
     */
    bool ReadBuf(T buff[], int len) {
        if (len > data_length_) {
            return false;
        }

        if (len <= 0) {
            return true;
        } else {
            data_mutex_.lock();
            for (int i(0); i < len; ++i) {
                buff[i] = data_buf_[i];
            }
            data_mutex_.unlock();
            return true;
        }
    }

    /**
     * delete several data from head of the queue
     * @param len
     * @return
     */
    bool DeletBuf(int len) {
        if (len > data_length_) {
            return false;
        } else {
            if (len < 0) {
                return false;
            } else {
                data_mutex_.lock();


                data_length_ -= len;
                memcpy(data_buf_,&data_buf_[len],data_length_);

                data_mutex_.unlock();
            }
        }
        return true;


    }

    inline int getSize() {
        int size  = 0;
//        data_mutex_.lock();
        size=data_length_;
//        data_mutex_.unlock();
        return size;
    }

private:
//    std::deque<T> data_buf_;
    T data_buf_[100000];
    int data_length_=0;


    std::mutex data_mutex_;


};


#endif //MINI_IMU_CHARQUEUE_H
