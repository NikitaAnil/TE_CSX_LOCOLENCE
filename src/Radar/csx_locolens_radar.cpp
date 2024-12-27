#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "Radar/csx_locolens_radar.h"

class VelocityInterface;


RadarInterface::RadarInterface()
{
    std::cout << "This is RadarInterface Constructor";
    //m_direction_flag_id = m_velocity_ptr -> getDirection();
    //m_speed_flag_id = m_velocity_ptr -> getSpeed();
    // write the logic for coverting this speed into diffrent modes
}


void RadarInterface::CollectCANDatafront(RingBuffer<CANData> &buffer)
{
    // Simulate front short camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (m_direction_flag_id == 1)
    {
        // do further implementation
    }
    std::cout << "Front short camera thread started." << std::endl;
}
void RadarInterface::CollectCANDataBack(RingBuffer<CANData> &buffer)
{
    // Simulate back short camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (m_direction_flag_id == 0)
    {
        // do further implementation
    }
    std::cout << "Back short camera thread started." << std::endl;
}

void RadarInterface::webApp_server_thread()
{
    // Simulate WebApp server operations and flag updates
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // // Simulate a flag change
    // std::lock_guard<std::mutex> lock(global_flags.mtx);
    // global_flags.trigger_flag = true; // Simulating flag update
    std::cout << "WebApp server thread completed." << std::endl;
}
void RadarInterface::processing_module_thread()
{
    // Simulate processing module that collects data from other threads
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Algorithm for object detection
    std::cout << "Processing module thread completed." << std::endl;
}
void RadarInterface::final_output_thread()
{
    // Final output processing after all modules have run
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Final output thread completed." << std::endl;
}