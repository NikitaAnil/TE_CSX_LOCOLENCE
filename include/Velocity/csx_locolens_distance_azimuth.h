/**
* @file     csx_locolens_distance_azimuth.h
* @author   Nikita Anil
* @brief    Header file for calculating the azimuth and distance between two geographic points.
*           REQ ID: SYSRS_010 - The LL-CDWS system shall have perception systems at 
*                               rear & front for detecting all obstacles in its path.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise self-localization 
*                               using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software modules 
*                               and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENCE_DISTANCE_AZIMUTH_H_
#define _CSX_LOCOLENCE_DISTANCE_AZIMUTH_H_

// system header file
#include <cmath>
#include <utility>

// local header file
#include "const_config.h"

class DistanceAzimuth {
public:
    constexpr double Rad2Deg(double rad);

    constexpr double Deg2Rad(double deg);

    double CalculateAzimuth(const std::pair<double, double>& point1,
        const std::pair<double, double>& point2);

    double Vincenty(const std::pair<double, double>& point1,
        const std::pair<double, double>& point2);
};

#endif // _CSX_LOCOLENCE_DISTANCE_AZIMUTH_H_
