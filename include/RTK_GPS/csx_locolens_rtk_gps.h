/**
* @file     csx_locolens_rtk_gps.h
* @author   Nikita Anil
* @brief    Header file for continuously sends RTCM data from the NTRIP caster to the RTK module and
*           pushes incoming GPS data into the buffer.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise
*                             self-localization using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software
*                               modules and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENS_RTK_GPS_H_
#define _CSX_LOCOLENS_RTK_GPS_H_

// system header file
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

// local header file
#include "const_config.h"
#include "RTK_GPS/csx_locolence_rtk_gps_recv.h"

class RtkGpsInterface 
{
public:
    RtkGpsInterface(); //constructor

    void ReadGPSDataFront(GPSBuffer& buffer, RtkModule module);

    void ReadGPSDataRear(GPSBuffer& buffer, RtkModule module);

private:
    // Flag to signal whether velocity is above threshold
    // to be used by camera selection thread
    std::atomic<bool> velocity_above_threshold { false };
    // Flag to signal direction of movement - to be used by sensor selection thread
    std::atomic<Direction> direction { STATIONARY };
    // create an instance of RtkGpsRecv class
    RtkGpsRecv rtk_gps_recv_instance;
};

#endif //_CSX_LOCOLENS_RTK_GPS_H_
