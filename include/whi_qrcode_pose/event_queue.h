/******************************************************************
eventQueue class for event notification between threads

Features:
- command pattern
- event queue to notify the comsumer
- can be used as blocking or non-blocking mode
- client can produce and consume specified event with name
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2021-07-06: Initial version
2021-08-29: Updated declaration and cleanup redundancy
2021-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <algorithm>
#include <list>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <assert.h>

template<typename T>
class EventQueue
{
public:
    static const std::string& VERSION()
    {
        return "00.08";
    }

public:
    using SharedPtr = std::shared_ptr<EventQueue>;
    using UniquePtr = std::unique_ptr<EventQueue>;
    using EventFunc = std::function<std::shared_ptr<T>()>; // nullary functor

public:
    struct stopped {};

    class EventPack
    {
    public:
        EventPack(EventFunc Func, const std::string Name = std::string("")) : name_(Name), func_(Func) {};
        ~EventPack() {};

    public:
        std::string name_;
        EventFunc func_;
    };

public:
    EventQueue(int MaxSize = 50, bool Blocking = false)
        : max_size_(MaxSize), blocking_(Blocking), state_(STATE_READY), ref_count_(0) {};
    ~EventQueue() { this->stop(false); };

    void produce(EventFunc Event, const std::string Name = std::string(""))
    {
        std::unique_lock<std::mutex> lock(mtx_);
        assert(STATE_READY == state_);
        if (queue_.size() == max_size_)
        {
            queue_.pop_front();
        }
        queue_.emplace_back(Event, Name);
        cv_.notify_one();
    };

    EventFunc consume()
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (blocking_)
        {
            cv_.wait(lock, [=]() { return STATE_READY != state_ || !queue_.empty(); });
            if (!queue_.empty())
            {
                EventFunc event(queue_.front().func_);
                queue_.pop_front();
                return event;
            }
            // The queue has been stopped. Notify the waiting thread blocked in
            // eventQueue::stop(true) (if any) that the queue is empty now
            cv_.notify_all();
            throw stopped();
        }
        else
        {
            if (!queue_.empty())
            {
                EventFunc event(queue_.front().func_);
                queue_.pop_front();
                return event;
            }
            else
            {
                return nullptr;
            }
        }
    };

    EventFunc consume(const std::string& Name)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        if (blocking_)
        {
            cv_.wait(lock, [=]() { return STATE_READY != state_ || !queue_.empty(); });
            if (!queue_.empty())
            {
                typename std::list<EventPack>::const_iterator found = std::find_if(queue_.begin(), queue_.end(), [=](const auto& Obj)
                {
                    return Obj.name_ == Name;
                });
                if (found != queue_.end())
                {
                    EventFunc event(found->func_);
                    queue_.erase(found);
                    return event;
                }
            }
            // The queue has been stopped. Notify the waiting thread blocked in
            // eventQueue::stop(true) (if any) that the queue is empty now
            cv_.notify_all();
            throw stopped();
        }
        else
        {
            if (!queue_.empty())
            {
                typename std::list<EventPack>::const_iterator found = std::find_if(queue_.begin(), queue_.end(), [=](const auto& Obj)
                {
                    return Obj.name_ == Name;
                });
                if (found != queue_.end())
                {
                    EventFunc event(found->func_);
                    queue_.erase(found);
                    return event;
                }
                else
                {
                    return nullptr;
                }
            }
            else
            {
                return nullptr;
            }
        }
    };

    void stop(bool WaitCompletion)
    {
        std::unique_lock<std::mutex> lock(mtx_);
        state_ = STATE_STOPPED;
        cv_.notify_all();
        if (WaitCompletion)
        {
            // Wait till all events have been consumed
            cv_.wait(lock, [=]() { return queue_.empty(); });
        }
        else
        {
            // Cancel all pending events
            queue_.clear();
        }
    };

protected:
    friend void intrusive_ptr_add_ref(EventQueue* Ptr)
    {
        ++Ptr->ref_count_;
    };

    friend void intrusive_ptr_release(EventQueue* Ptr)
    {
        if (!--Ptr->ref_count_)
        {
            delete Ptr;
        }
    };

public:
    enum State { STATE_READY, STATE_STOPPED };

protected:
    int max_size_{ -1 };
    bool blocking_{ false };
    std::mutex mtx_;
    std::condition_variable cv_;
    std::list<EventPack> queue_;
    State state_;
    std::atomic_int ref_count_;
};
