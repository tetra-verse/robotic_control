#ifndef __UTILS_BLOCKINGQUEUE_H_
#define __UTILS_BLOCKINGQUEUE_H_

#include <condition_variable>
#include <mutex>
#include <queue>

template <typename T>
class BlockingQueue
{
public:
    BlockingQueue() : data_(), mutex_(), condition_() {}

    BlockingQueue(BlockingQueue&) = delete;

    ~BlockingQueue() {}

    void push(T value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        data_.push(value);
        condition_.notify_all();
    }

    T take()
    {
        std::unique_lock<std::mutex> lock(mutex_);
        while (data_.empty()) {
            condition_.wait(lock);
        }

        T value(std::move(data_.front()));
        data_.pop();

        return value;
    }

    size_t size() const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return data_.size();
    }

private:
    std::queue<T> data_;

    std::mutex mutex_;
    std::condition_variable condition_;
};

#endif  // __UTILS_BLOCKINGQUEUE_H_
