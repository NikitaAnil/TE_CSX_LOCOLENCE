#ifndef CSX_CAMERA_H_
#define CSX_CAMERA_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "ring_buffer.h"
#include "Velocity/csx_locolens_velocity.h"

class CameraInterface {

public:
    CameraInterface(); //constructore
    // Function declarations for each thread's task
    void FrontShortRangeCam(RingBuffer<ImageData> &buffer);
    void FrontLongRangeCam(RingBuffer<ImageData> &buffer);
    void BackShortRangeCam(RingBuffer<ImageData> &buffer);
    void BackLongRangeCam(RingBuffer<ImageData> &buffer);

private:
    // created an instance to access the flags 
    std::shared_ptr<VelocityInterface> m_velocity_ptr;

    int m_direction_flag_id;
    int m_speed_flag_id;

protected:
};

#endif