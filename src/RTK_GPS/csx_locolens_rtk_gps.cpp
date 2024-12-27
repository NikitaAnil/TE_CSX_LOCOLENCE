#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

#include "RTK_GPS/csx_locolens_rtk_gps.h"

RtkGpsInterface::RtkGpsInterface()
{
    // Constructor of the RtkGpsInterface
}

void RtkGpsInterface:: RtkGpsFront(RingBufferGPS &gpsbuffer)
{
    // Simulate GPS data collection and flag updates for front
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "RTK_GPS_Front completed." << std::endl;
}

void RtkGpsInterface::RtkGpsBack(RingBufferGPS &gpsbuffer)
{
    // Simulate GPS data collection and flag updates for back
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "RTK_GPS_Back completed." << std::endl;
}
