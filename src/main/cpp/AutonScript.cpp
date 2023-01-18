#include "AutonScript.h"

using std::cout;
using std::endl;

AutonScript::AutonScript(std::string name)
{
    this->name = name;
}

AutonScript::~AutonScript()
{
    stop();
    if(thread != nullptr  && thread->joinable())
    {
        thread->join();
    }
}

void AutonScript::start()
{
    if(!running)
    {
        if(thread != nullptr && thread->joinable())
        {
            thread->join();//Very slight chance thread isn't fully dead, join just incase
        }
        thread = std::make_unique<std::thread>([this]()
        {
            running = true;
            cout << "Starting script \"" << name << "\"" << endl;
            this->script();
            cout << "Script \"" << name << "\" has ended" << endl;
        });
    }
}

void AutonScript::stop()
{
    running = false;
    kill_cv.notify_one();
    if(thread != nullptr  && thread->joinable())
    {
        thread->join();
    }
}


void AutonScript::sleep(std::chrono::microseconds delay)
{
    std::unique_lock<std::mutex> lck{mtx};
    kill_cv.wait_for(lck, delay);
}

void AutonScript::waitUntilTrue(std::function<bool()> lambda)
{
    while(true)
    {
        std::unique_lock<std::mutex> lck{mtx};
        kill_cv.wait_for(lck, wait_until_delay);
        if(!running)
        {
            break;
        }
        if(lambda())
        {
            break;
        }
    }
}

void AutonScript::waitUntilNear(std::function<double()> lambda, double target, double range)
{
    while(true)
    {
        std::unique_lock<std::mutex> lck{mtx};
        kill_cv.wait_for(lck, wait_until_delay);
        if(!running)
        {
            break;
        }
        double value = std::abs(lambda() - target);
        if(value <= range)
        {
            break;
        }
    }
}

/*
This code will help set each command in auton separately

Wait for: Exit the script early to prevent 




*/