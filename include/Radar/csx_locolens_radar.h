
#ifndef CSX_RADAR_H_
#define CSX_RADAR_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "const_config.h"
#include "Velocity/csx_locolens_velocity.h"

class RadarInterface {

public:
    RadarInterface(); //constructore
    // Function declarations for each thread's task
    void CollectCANDatafront(RingBuffer<CANData> &buffer);
    void CollectCANDataBack(RingBuffer<CANData> &buffer);

    void webApp_server_thread();
    void processing_module_thread();
    void final_output_thread();

private:
    // created an instance to access the flags 
    std::shared_ptr<VelocityInterface> m_velocity_ptr;

    int m_direction_flag_id;
    int m_speed_flag_id;

protected:
};
// Global shared data structure for flags and parameters

#endif

