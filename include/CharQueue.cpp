//
// Created by steve on 17-5-17.
//

#include "CharQueue.h"


bool CharQueue::AddBuf(char *buff, int len) {
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

bool CharQueue::ReadBuf(char *buff, int len) {

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


bool CharQueue::DeletBuf(int len) {
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
            }catch (std::exception &e)
            {
                std::cout << e.what()<<std::endl;
                std::cout << __FILE__ << ":" << __LINE__ << std::endl;
            }

            data_mutex_.unlock();
        }
    }


}