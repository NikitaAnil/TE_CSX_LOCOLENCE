/**
* @file     csx_locolens_switch_alert.h
* @author   Nikita Anil
* @brief    Header file for processing switch information, to provide an alert when a 
*           switch is within threshold.
*           REQ ID: SYSRS_070 - The LL-CDWS system shall utilize GIS data for 
*                               localizing track elements and assets.
*           REQ ID: SYSRS_030 - The LL-CDWS system shall detect turnout
*                               switches in the running track.
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENS_SWITCH_ALERT_H_
#define _CSX_LOCOLENS_SWITCH_ALERT_H_

// system header file
#include <atomic>
#include <map>
#include <utility>
#include <vector>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

//local header file
#include "Velocity/csx_locolens_distance_azimuth.h"
#include "const_config.h"

class SwitchAlert
{
public:
    SwitchAlert();
    void EvaluateSwitchDistances(const std::pair<double, double> &locomotive_coordinates,
                                 const std::vector<std::pair<double, double>> &switches,
                                 std::map<std::pair<double, double>, double> &switches_previous_distance);
    

    void SwitchAlertThread(GPSBuffer& gps_buffer, SwitchBuffer& switch_buffer);

private:
    // create an instance of DistanceAzimuth class
    DistanceAzimuth m_distance_azimuth_instance;
    
    // Flag to trigger the turnout switch detection thread
    std::atomic<bool> switch_distance_below_threshold { false };
};

#endif // _CSX_LOCOLENS_SWITCH_ALERT_H_