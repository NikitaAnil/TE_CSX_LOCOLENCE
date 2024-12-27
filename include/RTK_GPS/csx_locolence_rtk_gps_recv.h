/**
* @file     csx_locolence_rtk_gps_recv.h
* @author   Nikita Anil
* @brief    Header file for processing location data received from the RTK GPS modules.
*           REQ ID: SYSRS_060 - The LL-CDWS system shall provide precise
*                             self-localization using RTK correction services.
*           REQ ID: SYSRS_120 - The LL-CDWS system shall reuse customized software
*                               modules and hardware platforms from MoW CDWS project
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENS_RTK_GPS_RECV_H_
#define _CSX_LOCOLENS_RTK_GPS_RECV_H_

// system header file
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <netdb.h>
#include <cppcodec/base64_rfc4648.hpp>
#include <string>
#include <sstream>
#include <chrono>

class RtkGpsRecv
{
public:
    RtkGpsRecv();
    long double GetCurrentUnixTimestamp();
    int ConnectToNTRIPCaster(const std::string &ntrip_caster_host,
                            const int ntrip_caster_port,
                            const std::string &username,
                            const std::string &password,
                            const std::string &mount_point);

    ssize_t RecvWithTimeout(int sockfd, void *buf, size_t len,
                            int flags, double timeout_sec);

    void ParseGNGLL(const std::string &nmea_sentence,
                    double &latitude, double &longitude);

private:
    int ntrip_caster_socket; // Socket for the NTRIP caster
};
#endif // _CSX_LOCOLENS_RTK_GPS_RECV_H_