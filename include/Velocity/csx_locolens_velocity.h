/**
* @file     csx_locolens_velocity.h
* @author   Nikita Anil
* @brief    Header file for calculating the velocity of the locomotive and the direction
*           in which the locomotive is travelling.
*           REQ ID: SYSRS_010 - The LL-CDWS system shall have perception systems at 
*                               rear & front for detecting all obstacles in its path.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise self-localization 
*                               using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software modules 
*                               and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENCE_VELOCITY_H_
#define _CSX_LOCOLENCE_VELOCITY_H_

// system header file
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

//system local file
#include "Velocity/csx_locolens_distance_azimuth.h"
#include "const_config.h"

class VelocityInterface {
public:
    VelocityInterface();
    void VelocityLoop(GPSBuffer& buffer);
    void DirectionVelocityThread(GPSBuffer& front, GPSBuffer& rear);
    Direction FindDirection(std::pair<double, double> front_old,
        std::pair<double, double> back_old,
        std::pair<double, double> front_new);

    double CalculateVelocity(const std::pair<double, double>& point1,
        const std::pair<double, double>& point2,
        double time_1, double time_2, double& azimuth);
private:
    // create an instance of DistanceAzimuth class
    DistanceAzimuth distance_azimuth_instance;
};

#endif // _CSX_LOCOLENCE_VELOCITY_H_
