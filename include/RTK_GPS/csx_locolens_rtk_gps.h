#ifndef CSX_RTK_GPS_H_
#define CSX_RTK_GPS_H_
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "ring_buffer.h"

class RtkGpsInterface {

public:
    RtkGpsInterface(); //constructor
    // Function declarations for each thread's task
    void RtkGpsFront(RingBufferGPS &gpsbuffer);
    void RtkGpsBack(RingBufferGPS &gpsbuffer);
};

#endif