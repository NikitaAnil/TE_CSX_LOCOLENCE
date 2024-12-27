#ifndef CSX_WEBAPP_H_
#define CSX_WEBAPP_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

class WebAppInterface
{
    public:
    WebAppInterface();
    // member function
    void WebAppThread();
    void SetSwitchTriggerFlag();
    double GetSwitchTriggerFlag();

    private:
    // data member
    double switch_trigger_flag; 
};

#endif