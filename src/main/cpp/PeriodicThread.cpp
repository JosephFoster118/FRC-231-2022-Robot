#include "PeriodicThread.h"



PeriodicThread::PeriodicThread(std::chrono::microseconds thread_period, std::string thread_name, std::function<void()> lambda)
{
    std::cout << "Created thread [" << thread_name << "]\n";
    period = thread_period;
    name = thread_name;
    run_lambda = lambda;
}


PeriodicThread::~PeriodicThread()
{
    stop();
    if(thread != nullptr  && thread->joinable())
    {
        thread->join();
    }
}

void PeriodicThread::start()
{
    if(!running)
    {
        if(thread != nullptr && thread->joinable())
        {
            thread->join();//Very slight chance thread isn't fully dead, join just incase
        }
        thread = std::make_unique<std::thread>([this]()
        {
            this->run();
        });
    }
}

void PeriodicThread::run()
{
    running = true;
    auto start_time = std::chrono::high_resolution_clock::now();
    while(running)
    {
        run_lambda();//Run the lambda passed in
        auto end_time = std::chrono::high_resolution_clock::now();
        auto time_passed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        if(time_passed > period)//Period was passed, print warning and set start_time to current time
        {
            std::cout << "Thread \"" << name << "\"exceeded its period!\n";
            start_time = std::chrono::high_resolution_clock::now();
        }
        else
        {
            std::unique_lock<std::mutex> lck{mtx};
            kill_cv.wait_for(lck, period - time_passed);
            start_time += period;
        }
    }
    std::cout << "Thread \"" << name << "\" has died\n";
}

void PeriodicThread::stop()
{
    running = false;
    kill_cv.notify_one();
    if(thread != nullptr  && thread->joinable())
    {
        thread->join();
    }
}

bool PeriodicThread::isRunning()
{
    return running;
}
