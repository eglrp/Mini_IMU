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
 */
template<typename T>
class CharQueue {

public:
    CharQueue(int size = 10000) {
        try {

            data_buf_.resize(size);
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

            for (int i(0); i < len; ++i) {
                data_buf_.push_back(buff[i]);
            }
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
        if (len > data_buf_.size()) {
            return false;
        }

        if (len <= 0) {
            return true;
        } else {
            data_mutex_.lock();
            for (int i(0); i < len; ++i) {
                buff[i] = data_buf_.at(i);
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
        if (len > data_buf_.size()) {
            return false;
        } else {
            if (len < 0) {
                return false;
            } else {
                data_mutex_.lock();
                try {
                    for (int i(0); i < len; ++i) {
                        data_buf_.pop_front();
                    }
                } catch (std::exception &e) {
                    std::cout << e.what() << std::endl;
                    std::cout << __FILE__ << ":" << __LINE__ << std::endl;
                }

                data_mutex_.unlock();
            }
        }


    }

    inline int getSize() {
        return data_buf_.size();
    }

private:
    std::deque<T> data_buf_;

    std::mutex data_mutex_;


};


#endif //MINI_IMU_CHARQUEUE_H
