#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "Velocity/csx_locolens_velocity.h"

VelocityInterface::VelocityInterface()
{
    //constructor of the VelocityInterface class
}

void VelocityInterface::CalculateVelocity(RingBufferGPS &gpsbuffer)
{
// Simulate velocity calculation and flag updates
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Velocity thread completed." << std::endl;
    // Processing logic to handle flags and modules
    // on the base of gps buffer will calculate the direction and speed of the locomotive
    // will set that values with VelocityInterface class members and 
    // so other class can access this values through the getter funtions
}

double VelocityInterface::getDirection()
{
    return m_direction_flag;
}

double VelocityInterface::getSpeed()
{
    return m_speed_flag;
}