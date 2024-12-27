#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "Camera/csx_locolens_camera.h"

class VelocityInterface;

CameraInterface::CameraInterface()
{
    std::cout << "This is CameraInterface Constructor";
    m_direction_flag_id = m_velocity_ptr -> getDirection();
    m_speed_flag_id = m_velocity_ptr -> getSpeed();
    // write the logic for coverting this speed into diffrent modes
}

void CameraInterface::FrontShortRangeCam(RingBuffer<ImageData> &buffer)
{
    // Simulate front short camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Front short camera thread started." << std::endl;
}

void CameraInterface::FrontLongRangeCam(RingBuffer<ImageData> &buffer)
{
    // Simulate front long camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Front long camera thread started." << std::endl;
}

void CameraInterface::BackShortRangeCam(RingBuffer<ImageData> &buffer)
{
    // Simulate back short camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Back short camera thread started." << std::endl;
}

void CameraInterface::BackLongRangeCam(RingBuffer<ImageData> &buffer)
{
    // Simulate back long camera operations
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "Back long camera thread started." << std::endl;
}

