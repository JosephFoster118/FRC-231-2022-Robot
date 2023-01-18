#pragma once
#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <cmath>
#include <iostream>

#define SCRIPT_RUN_CHECK if(!running) return; //Deifine a macro (Anywhere you put the code, (If not running return); 

class AutonScript
{
public:
    AutonScript() = delete;
    AutonScript(const AutonScript&) = delete;//Removes the copy constructor
    AutonScript(std::string name);
    virtual ~AutonScript();


    void start();//Starts the thread
    void stop();//Stop the thread
protected:
    const std::chrono::milliseconds wait_until_delay{5};
    void sleep(std::chrono::microseconds delay);
    void waitUntilTrue(std::function<bool()> lambda);
    void waitUntilNear(std::function<double()> lambda, double target, double range = 0.0001);
    std::string name;
    virtual void script() = 0; //You must include it in the child (pure virutal function)
    std::atomic<bool> running{false};
private:
    std::unique_ptr<std::thread> thread;
    std::condition_variable kill_cv;//A condition_variable that allows early canceling of the thread
    std::mutex mtx;//Mutex used for condition wait
};




