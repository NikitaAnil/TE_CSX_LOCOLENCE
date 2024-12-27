
/**
* @file     csx_locolens_rtk_gps.cpp
* @author   Nikita Anil
* @brief    Continuously sends RTCM data from the NTRIP caster to the RTK module and
*           pushes incoming GPS data into the buffer.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise
*                             self-localization using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software
*                               modules and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

// system header file
#define SIMULATION
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <libserial/SerialStream.h>
#include <mutex>
#include <thread>

// local header file
#include "RTK_GPS/csx_locolens_rtk_gps.h"

RtkGpsInterface::RtkGpsInterface()
{
    // Constructor of the RtkGpsInterface
}

/**
* @brief - This function runs an infinite loop that retrieves RTCM data from the NTRIP
*          caster socket. Upon successfully receiving data, it writes the data to a
*          serial port and checks for GNGLL sentences in the response. If found, it
*          extracts the latitude and longitude, and appends the data to the GPS buffer.
* @param   buffer - A reference to the GPS buffer where GPS data will be stored.
* @param   module - The RTK GPS module to be used for obtaining location data.
* @return  void
*/
#ifndef SIMULATION
void RtkGpsInterface::ReadGPSDataFront(GPSBuffer& buffer, RtkModule module)
{
    LibSerial::SerialStream rtk_serial;
    // Connect to the respective RTK GPS module
    try 
    {
        if (module == FRONT) 
        {
            rtk_serial.Open(kRtkportFront);
        } 
        else 
        {
            rtk_serial.Open(kRtkportRear);
        }
        rtk_serial.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

        while (true) 
        {
            char buf[4096];
            // Receive the NTRIP correction data
            int bytes_received = RecvWithTimeout(ntrip_caster_socket,
                buf, sizeof(buf), 0, 1.0);
            if (bytes_received > 0) 
            {
                try
                {
                    // Write the correction to GPS module
                    rtk_serial.write(buf, bytes_received);
                    char response[10240];
                    // Get the GPS messages from GPS module
                    rtk_serial.readsome(response, sizeof(response));
                    std::streamsize bytesRead = rtk_serial.gcount();
                    std::string str(response, bytesRead);

                    size_t start = 0;
                    size_t end = 0;
                    // Find the line with the string "$GNGLL" - This line
                    // contains the necessary location information
                    while ((end = str.find('\n', start)) != std::string::npos) 
                    {
                        std::string line = str.substr(start, end - start);
                        start = end + 1;
                        if (line.find("$GNGLL") != std::string::npos) 
                        {
                            double latitude, longitude;
                            ParseGNGLL(line, latitude, longitude);
                            long double timestamp = GetCurrentUnixTimestamp();
                            // Write the coordinates and time to the GPS buffer
                            buffer.Push(timestamp, { latitude, longitude });
                        }
                    }
                } 
                catch (const std::exception& e) 
                {
                    std::cerr << "Failed to write: " << e.what() << std::endl;
                }
            } 
            else if (bytes_received == 0) 
            {
                std::cerr << "Connection closed by the server." << std::endl;
                close(ntrip_caster_socket);
            } 
            else 
            {
                try 
                {
                    std::cerr << "Timeout occurred." << std::endl;
                    throw std::runtime_error("RTCM Data not received");
                } 
                catch (std::runtime_error& e) 
                {
                    std::cerr << "Buffer update stopped: " << e.what() << '\n';
                    //sleep for 10 seconds and try again
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                }
            }
        }
    } 
    catch (const LibSerial::OpenFailed& e) 
    {
        std::cerr << "Error opening RTK GPS Port:" << e.what() << '\n';
        std::cerr << "Buffer update failed" << std::endl;
    } 
    catch (std::exception& e) 
    {
        std::cerr << "Buffer update stopped: " << e.what() << '\n';
    }
}
#endif // SIMULATION

/**
* @brief - This function runs an infinite loop that retrieves RTCM data from the NTRIP
*          caster socket. Upon successfully receiving data, it writes the data to a
*          serial port and checks for GNGLL sentences in the response. If found, it
*          extracts the latitude and longitude, and appends the data to the GPS buffer.
* @param   buffer - A reference to the GPS buffer where GPS data will be stored.
* @param   module - The RTK GPS module to be used for obtaining location data.
* @return  void
*/
#ifdef SIMULATION
#include <fstream>
void RtkGpsInterface::ReadGPSDataRear(GPSBuffer& buffer, RtkModule module)
{
    std::string filename = "";
    long double timestamp;
    if (module == FRONT) {
        filename = "../gps_front.txt";
    } else {
        filename = "../gps_rear.txt";
    }
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double latitude, longitude;
        char comma;
        if (ss >> latitude >> comma >> longitude) {
            timestamp = rtk_gps_recv_instance.GetCurrentUnixTimestamp();
            buffer.Push(timestamp, { latitude, longitude });
            std::this_thread::sleep_for(std::chrono::seconds(2));
        } else {
            std::cerr << "Invalid data format in file: " << line << std::endl;
        }
    }
    try {
        throw std::runtime_error("End of file reached");
    } catch (const std::exception& e) {
        std::cerr << "Buffer update stopped: " << e.what() << '\n';
    }
}
#endif // SIMULATION
