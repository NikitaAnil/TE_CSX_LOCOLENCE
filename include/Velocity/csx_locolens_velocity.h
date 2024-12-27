#ifndef VELOCITY_INTERFACE_H_
#define VELOCITY_INTERFACE_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "ring_buffer.h"


class VelocityInterface
{
    public:
    VelocityInterface(); //constructor
    // Function declarations for each thread's task
    void CalculateVelocity(RingBufferGPS &gpsbuffer);
    double getDirection();
    double getSpeed();

    private:
    double m_direction_flag = 1; // 0 for back, 1 for front
    double m_speed_flag; // 
};

#endif