#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "Processing_Module/csx_locolens_processing_module.h"
#include "Radar/csx_locolens_radar.h"
#include "Camera/csx_locolens_camera.h"

ProcessingModuleInterface::ProcessingModuleInterface()
{
    // constructor ProcessingModuleInterface
}

void ProcessingModuleInterface::ProcessModule()
{
    // Simulate processing module
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Processing Module Started." << std::endl;
}

void ProcessingModuleInterface::FinalOutput()
{
    // Simulate processing module
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Final Module Started." << std::endl;
}