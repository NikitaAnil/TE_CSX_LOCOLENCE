/**
* @file     csx_locolence_rtk_gps_recv.cpp
* @author   Nikita Anil
* @brief    This file defines various functions to read RTCM messages from the NTRIP
*           caster, write the messages to the GPS modules and to obtain and location
*           information from the RTK GPS modules. The parsed location information and
*           the current timestamp are then pushed to the GPS buffer for further
*           processing: for direction determinationand velocity determination, to update
*           the webapp and for swich alert monitoring.
*           This uses the cppcodec library and the LibSerial library.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise
*                             self-localization using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software
*                               modules and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

// system header file
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
//#include <cppcodec/base64_rfc4648.hpp>
#include <string>
#include <sstream>
#include <chrono>

// local header file
#include "RTK_GPS/csx_locolence_rtk_gps_recv.h"

RtkGpsRecv::RtkGpsRecv()
{
    //constructor for initializing class data memebrs and objec creation
}

/**
* @brief - This function uses the system clock to obtain the current time since the
*          Unix epoch (January 1, 1970). It converts this time into nanoseconds and
*          returns the result as a long double representing the timestamp in seconds.
* @return  The current Unix timestamp in seconds.
*/
long double RtkGpsRecv::GetCurrentUnixTimestamp()
{
    // Get the current time point
    auto now = std::chrono::system_clock::now();
    auto now_us = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch());
    // Return the current Unix timestamp in seconds
    return now_us.count() / 1000000000.0;
}


/**
* @brief - This function creates a socket, resolves the hostname of the NTRIP caster,
*          and connects to it using the provided credentials and mount point. It also
*          sends an NTRIP request for RTCM data.
* @param   ntrip_caster_host: The hostname or IP address of the NTRIP caster.
* @param   ntrip_caster_port: The port number on which the NTRIP caster is listening.
* @param   username: The username for authentication with the NTRIP caster.
* @param   password: The password for authentication with the NTRIP caster.
* @param   mount_point: The mount point for the RTCM data stream.
* @return  The NTRIP socket descriptor.
*/
int RtkGpsRecv::ConnectToNTRIPCaster(const std::string &ntrip_caster_host,
                         const int ntrip_caster_port,
                         const std::string &username,
                         const std::string &password,
                         const std::string &mount_point)
{
    // Create socket
    ntrip_caster_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (ntrip_caster_socket == -1)
    {
        std::cerr << "Can't create socket! Quitting" << std::endl;
        return -1;
    }

    // Set up server address structure
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(ntrip_caster_port);

    // Resolve hostname
    struct hostent *host = gethostbyname(ntrip_caster_host.c_str());
    if (host == nullptr)
    {
        std::cerr << "Error resolving hostname." << std::endl;
        close(ntrip_caster_socket);
        return -1;
    }
    memcpy(&server.sin_addr, host->h_addr_list[0], host->h_length);

    // Connect to the NTRIP Caster
    std::cout << "Connecting to the NTRIP Caster..." << std::endl;
    if (connect(ntrip_caster_socket,
                reinterpret_cast<struct sockaddr*>(&server),
                sizeof(server)) == -1)
    {
        std::cerr << "Can't connect to NTRIP Caster! Quitting" << std::endl;
        close(ntrip_caster_socket);
        return -1;
    }
    std::cout << "Connected to the NTRIP Caster." << std::endl;

    // Prepare and send the NTRIP request
    std::string encoded_credentials = username + ":" + password;
    std::string encoded_string =
        cppcodec::base64_rfc4648::encode(encoded_credentials);

    std::cout << "Sending NTRIP Caster request..." << std::endl;
    std::string request = "GET /" + mount_point + " HTTP/1.1\r\n" +
                          "Host: " + ntrip_caster_host + ":" +
                          std::to_string(ntrip_caster_port) + "\r\n" +
                          "Ntrip-Version: Ntrip/2.0\r\n" +
                          "User-Agent: NTRIP client\r\n" +
                          "Authorization: Basic " + encoded_string + "\r\n" +
                          "\r\n";
    send(ntrip_caster_socket, request.c_str(), request.size(), 0);
    std::cout << "NTRIP Caster request sent." << std::endl;

    std::cout << "Waiting for RTCM messages..." << std::endl;

    // Return the socket descriptor for further use
    return ntrip_caster_socket;
}

/**
* @brief - This function attempts to read data from the specified socket `sockfd` into
*          the provided buffer `buf` for a maximum duration defined by `timeout_sec`.
*          It utilizes the `select()` system call to implement the timeout functionality.
* @param   sockfd: The file descriptor of the socket from which data will be received.
* @param   buf: A pointer to the buffer where the received data will be stored.
* @param   len: The maximum number of bytes to read from the socket.
* @param   flags: Flags that control the behavior of the `recv()` function.
* @param   timeout_sec: The maximum time, in seconds, to wait for data to arrive.
* @return  The number of bytes received on success, -2 on timeout, and -1 on error.
*/
ssize_t RtkGpsRecv::RecvWithTimeout(int sockfd, void *buf, size_t len,
                        int flags, double timeout_sec)
{
    // File descriptor set, needed for select()
    fd_set read_fds;
    struct timeval timeout;
    ssize_t bytes_received;

    FD_ZERO(&read_fds);
    FD_SET(sockfd, &read_fds);

    // Set the timeout as per user value - convert double value into
    // seconds and microseconds
    timeout.tv_sec = static_cast<int>(timeout_sec);
    timeout.tv_usec = (timeout_sec - static_cast<int>(timeout_sec)) * 1000000;

    // Wait till socket is ready to read from, or timeout occurs,
    // whichever happens first
    int retval = select(sockfd + 1, &read_fds, NULL, NULL, &timeout);

    if (retval == -1)
    {
        perror("select");
        return -1;
    }
    else if (retval == 0)
    {
        // Timeout condition
        return -2;
    }
    else
    {
        // Data is available to read from the socket
        bytes_received = recv(sockfd, buf, len, flags);
        if (bytes_received < 0)
        {
            perror("recv");
        }
        return bytes_received;
    }
}

/**
* @brief - This function takes an NMEA GNGLL sentence as input, extracts the latitude
*          and longitude values, and converts them to decimal degrees. It throws an
*          exception if the GPS signal is lost.
* @param   nmea_sentence: The GNGLL NMEA sentence containing the latitude and
*                       longitude information.
* @param   latitude: A reference to a double where the parsed latitude will be stored.
* @param   longitude: A reference to a double where the parsed  longitude will be stored.
* @return  void
*/
void RtkGpsRecv::ParseGNGLL(const std::string &nmea_sentence,
                double &latitude, double &longitude)
{
    std::stringstream stream_of_string(nmea_sentence);
    std::string token;

    // Skip the sentence identifier
    std::getline(stream_of_string, token, ',');

    // Parse latitude
    std::getline(stream_of_string, token, ',');
    if (token.empty())
    {
        throw std::runtime_error("GPS signal lost");
    }

    size_t en = 0;
    // Remove the trailing period, if present
    if (token[token.length() - 1] == '.')
    {
        token = token.substr(0, token.length() - 1);
    }
    // Remove any trailing characters after `*`, if present
    else if ((en = token.find('*')) != std::string::npos)
    {
        token = token.substr(0, en);
    }
    // Convert latitude from degrees and minutes to decimal
    double latitude_degree = std::stod(token.substr(0, 2));
    double latitude_minute = std::stod(token.substr(2));

    char lat_dir;
    std::getline(stream_of_string, token, ',');
    lat_dir = token[0];

    latitude = latitude_degree + (latitude_minute / 60.0);
    // Convert the latitude to negative, if direction is South
    if (lat_dir == 'S')
    {
        latitude = -latitude;
    }

    // Parse longitude
    std::getline(stream_of_string, token, ',');
    if (token.empty())
    {
        throw std::runtime_error("GPS signal lost");
    }
    // Remove the trailing period, if present
    if (token[token.length() - 1] == '.')
    {
        token = token.substr(0, token.length() - 1);
    }
    // Convert longitude from degrees and minutes to decimal
    double longitude_degree = std::stod(token.substr(0, 3));
    double longitude_minute = std::stod(token.substr(3));

    char longitude_token;
    std::getline(stream_of_string, token, ',');
    longitude_token = token[0];

    longitude = longitude_degree + (longitude_minute / 60.0);
    // Convert the longitude to negative, if direction is West
    if (longitude_token == 'W')
    {
        longitude = -longitude;
    }
}
