#pragma once
#include <string>
#include <cstdint>
#include <chrono>
#include <iostream>
#include <thread>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>


class PeriodicThread
{
public:
    PeriodicThread() = delete;//Removes the default constructor
    PeriodicThread(const PeriodicThread&) = delete;//Removes the copy constructor
    //Todo: delete other unused constructors
    PeriodicThread(std::chrono::microseconds thread_period, std::string thread_name, std::function<void()> lambda);
    virtual ~PeriodicThread();
    bool isRunning();

    void start();//Starts the thread
    void stop();//Stop the thread

private:
    std::unique_ptr<std::thread> thread;
    void run();

    std::function<void()> run_lambda;

    std::condition_variable kill_cv;//A condition_variable that allows early canceling of the thread
    std::mutex mtx;//Mutex used for condition wait
    std::atomic<bool> running{false};

    std::chrono::microseconds period;
    std::string name;

};



