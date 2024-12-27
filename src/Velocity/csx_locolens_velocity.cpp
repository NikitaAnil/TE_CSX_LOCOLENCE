/**
* @file     csx_locolens_velocity.cpp
* @author   Nikita Anil
* @brief    This file defines functions to determine the direction of movement of the
*           locomotive, and to compute the velocity of the locomotive. The direction is
*           determined using the distance between two instances of position of the
*           locomotive. The distance is computed using the Vincenty's formula.
*           in which the locomotive is travelling.
*           REQ ID: SYSRS_010 - The LL-CDWS system shall have perception systems at 
*                               rear & front for detecting all obstacles in its path.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise self-localization 
*                               using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software modules 
*                               and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

// system header file
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

// local header file
#include "Velocity/csx_locolens_distance_azimuth.h"
#include "Velocity/csx_locolens_velocity.h"
#include "const_config.h"

/**
* @brief - Constructor of the Velocity class to initialize class members
* @param   NA
* @return  NA
*/

VelocityInterface::VelocityInterface()
{
    //constructor of the VelocityInterface class
}

/**
* @brief - Thread determining the direction in which the locomotive is travelling, then
*          repeatedly checks whether the velocity is within the threshold.
* @param   front: A reference to the `GPSBuffer` object to obtain the current
*                 location of the front of the locomotive.
* @param   rear: A reference to the `GPSBuffer` object to obtain the current
*                location of the rear of the locomotive.
* @return  A `Direction` enumeration indicating the locomotive's movement direction.
*/
void VelocityInterface::DirectionVelocityThread(GPSBuffer& front, GPSBuffer& rear)
{
    while (direction.load() == STATIONARY) 
    {
        auto front_old = std::get<1>(front.Pop());
        auto rear_old = std::get<1>(rear.Pop());
        auto front_new = std::get<1>(front.Pop());
        if (front_old.first != 0 && front_old.second != 0 && front_new.first != 0 && front_new.second != 0 && rear_old.first != 0 && rear_old.second != 0) 
        {
            Direction dir = FindDirection(front_old, rear_old, front_new);
            direction.store(dir);
        }
    }
    if (direction.load() == FORWARD) 
    {
        std::cout << "Direction: Forwards" << std::endl;
        // Use the RTK GPS from the front for velocity calculations
        VelocityLoop(front);
    } 
    else if (direction.load() == REVERSE) 
    {
        std::cout << "Direction: Reverse" << std::endl;
        // Use the RTK GPS from the rear for velocity calculations
        VelocityLoop(rear);
    }
}

/**
* @brief - Repeatedly calculates the velocity between two points to check if it crosses
*          the threshold, while also determining the azimuth angle.
* @param   buffer: A reference to the `GPSBuffer` object to obtain the current
*                location of the locomotive.
* @return  void
*/
void VelocityInterface::VelocityLoop(GPSBuffer& buffer)
{
    auto previous_coordinates = buffer.Pop();
    while (true) 
    {
        auto new_coordinates = buffer.Pop();
        auto new_timestamp = std::get<0>(new_coordinates);
        auto previous_timestamp = std::get<0>(previous_coordinates);
        auto new_coord = std::get<1>(new_coordinates);
        auto previous_coord = std::get<1>(previous_coordinates);

        // Check if the coordinates are valid
        if (new_coord.first != 0.0 && new_coord.second != 0.0 && previous_coord.first != 0.0 && previous_coord.second != 0.0) 
        {
            double azimuth;
            double velocity;
            velocity = CalculateVelocity(previous_coord, new_coord,
                previous_timestamp, new_timestamp,
                azimuth);
            // Compare the velocity obtained and set the flag
            if (velocity >= kVelocityThreshold) 
            {
                velocity_above_threshold.store(true);
            } 
            else 
            {
                velocity_above_threshold.store(false);
            }
            std::cout << "Velocity: " << velocity << " m/s" << std::endl;
            std::cout << "Azimuth: " << azimuth << " degrees" << std::endl;
        }
        previous_coordinates = new_coordinates;
        // arbitrary sleep added for testing
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
}

/**
* @brief - Determines the direction in which the locomotive is travelling.
* @param   front_old: A pair of the latitude and longitude of the locomotive's
*                     front position at the initial time in degrees.
* @param   back_old: A pair of the latitude and longitude of the locomotive's
*                    rear position at the initial time in degrees.
* @param   front_new: A pair of the latitude and longitude of the locomotive's
*                     front position at the final time in degrees.
* @return  A `Direction` enumeration indicating the locomotive's movement direction.
*/

Direction VelocityInterface::FindDirection(std::pair<double, double> front_old,
    std::pair<double, double> back_old,
    std::pair<double, double> front_new)
{
    Direction dir;
    // Distance between front and back of the locomotive
    double distance1 = distance_azimuth_instance.Vincenty(front_old, back_old);
    // Relative distance between the front of the locomotive after movement,
    // and the back of the locomotive initially
    double distance2 = distance_azimuth_instance.Vincenty(front_new, back_old);
    if (distance1 < distance2) 
    {
        dir = FORWARD;
    } 
    else if (distance1 > distance2) 
    {
        dir = REVERSE;
    } 
    else 
    {
        dir = STATIONARY;
    }
    return dir;
}

/**
* @brief - Calculates the velocity between two geographic points based on distance and 
*          time interval, while also determining the azimuth angle.
* @param point1: A pair of double values of the latitude and longitude of the
*                first point in degrees.
* @param point2: A pair of double values of the latitude and longitude of the
*                second point in degrees.
* @param time_1: The timestamp at the first point in seconds.
* @param time_2: The timestamp at the second point in seconds.
* @param azimuth: A reference to a double variable where the azimuth angle in
*                 degrees will be stored.
* @return  The velocity between the two points in meters per second.
*/

double VelocityInterface::CalculateVelocity(const std::pair<double, double>& point1,
    const std::pair<double, double>& point2,
    double time_1, double time_2, double& azimuth)
{
    double delta_time = time_2 - time_1;
    if (delta_time <= 0) 
    {
        throw std::invalid_argument("Time difference must be positive.");
    }

    azimuth = distance_azimuth_instance.CalculateAzimuth(point1, point2);
    double distance = distance_azimuth_instance.Vincenty(point1, point2);
    return distance / delta_time;
}
