/**
* @file     csx_locolens_switch_alert.h
* @author   Nikita Anil
* @brief    This file defines various functions and classes for managing MQTT
*           communication with an asynchronous client. It includes functions to handle
*           MQTT callbacks for received messages and publishing GPS data to the server.
*           The messages are received as a JSON-formatted message, and the necessary
*           switch information is read from the message and pushed to the switch buffer.
*           This uses the nlohmann JSON library and the paho.mqtt.c++ library.
*           REQ ID: SYSRS_070 - The LL-CDWS system shall utilize GIS data for 
*                               localizing track elements and assets.
*           REQ ID: SYSRS_030 - The LL-CDWS system shall detect turnout
*                               switches in the running track.
* @copyright  Tata Elxsi (c) Copyright 2024
*/

// system header file
#include <atomic>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

// local header file
#include "WebApp/csx_locolens_switch_alert.h"

SwitchAlert::SwitchAlert()
{

}

/**
* @brief -   This function calculates the distance to each switch from the locomotive
*            and compares it to the previous measured distance from the same switch. If
*            the switch is getting closer to the locomotive, and falls within the
*            threshold, it generates an alert, by setting the flag 
*            switch_distance_below_threshold`. It also updates the measured distance to
*            the switch in a map, for identifying if the switch is getting closer to the
*            locomotive or not. The map is reset everytime new switch data is received
*            from the webapp.
* @param     locomotive_coordinates: The current location of the locomotive.
* @param     switches: The vector of switches that has been identified by the webapp.
* @param     switches_previous_distance: The map of switches and its distance
*                                    measured previously by the function.
* @return    void
*/
void SwitchAlert::EvaluateSwitchDistances(const std::pair<double, double> &locomotive_coordinates,
                                          const std::vector<std::pair<double, double>> &switches,
                                          std::map<std::pair<double, double>, double> &switches_previous_distance)
{
    for (std::pair<double, double> sw : switches)
    {
        double distance_to_switch = m_distance_azimuth_instance.Vincenty(locomotive_coordinates, sw);
        auto switch_prev_distance = switches_previous_distance.find(sw);
        if (switch_prev_distance != switches_previous_distance.end())
        {
            double distance_to_switch_previous = switch_prev_distance->second;
            if (distance_to_switch < distance_to_switch_previous &&
                distance_to_switch < kSwitchDistanceThreshold)
            {
                std::cout << "Switch Alert: (" << sw.first << ", "
                          << sw.second << ") is within threshold" << std::endl;
                std::cout << "Distance to switch: "
                          << distance_to_switch << std::endl;
                switch_distance_below_threshold.store(true);
            }
        }
        switches_previous_distance[sw] = distance_to_switch;
    }
}

/**
* @brief -   This function repeatedly checks for any switches that are approaching the
*            locomotive and within the threshold, if the alert has been reset. Once a
*            switch satisfies the criteria, an alert is generated and the function waits
*            until the flag is reset.
* @param     gps_buffer: A reference to the `GPSBuffer` object to obtain the current
*                    location of the locomotive.
* @param     switch_buffer: A reference to the `SwitchBuffer` object to store the
*                           coordinates of the switches.
* @return    void
*/
void SwitchAlert::SwitchAlertThread(GPSBuffer& gps_buffer, SwitchBuffer& switch_buffer)
{
    std::vector<std::pair<double, double>> switches;
    //std::vector<std::pair<double, double>> switches_previous;
    std::map<std::pair<double, double>, double> switches_previous_distance;
    while (true) 
    {

        if (switch_distance_below_threshold.load() == false) 
        {
            if (switch_buffer.CheckForNewData() == true) 
            {
                switches_previous_distance.clear();
                //switches_previous = switches;
                switches = std::move(switch_buffer.Pop());
            }
            auto locomotive_coordinates = std::get<1>(gps_buffer.Pop());
            EvaluateSwitchDistances(locomotive_coordinates,
                switches, switches_previous_distance);
        }
    }
}