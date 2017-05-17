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
    bool AddBuf(char buff[], int len);

    /**
     * Read first several data from queue.
     * @param buff  point for buff
     * @param len length
     * @return
     */
    bool ReadBuf(char *buff, int len);

    /**
     * delete several data from head of the queue
     * @param len
     * @return
     */
    bool DeletBuf(int len);

private:
    std::deque<char> data_buf_;

    std::mutex data_mutex_;


};


#endif //MINI_IMU_CHARQUEUE_H
