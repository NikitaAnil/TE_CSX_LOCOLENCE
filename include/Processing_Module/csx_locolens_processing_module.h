#ifndef CSX_PROCESSING_MODULE_H_
#define CSX_PROCESSING_MODULE_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

class ProcessingModuleInterface
{
    public:
    // constructor
    ProcessingModuleInterface();
    // member function of the class
    void ProcessModule();
    // object detection
    // sensor fusion
    // object tracking
    // collision estimation
    // switch alignment inspection
    // Derail block detection

    void FinalOutput();
    
    private:
    // data members of the class

};

#endif