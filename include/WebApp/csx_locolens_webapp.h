/**
* @file     csx_locolens_webapp.h
* @author   Nikita Anil
* @brief    Header file for handling MQTT client interactions.
*           REQ ID: SYSRS_070 - The LL-CDWS system shall utilize GIS data for 
*                               localizing track elements and assets.
*           REQ ID: SYSRS_030 - The LL-CDWS system shall detect turnout
*                               switches in the running track.
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CSX_LOCOLENS_WEBAPP_H_
#define _CSX_LOCOLENS_WEBAPP_H_

// system header file
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <utility>
#include <vector>

//local header file
#include "const_config.h"
#include "WebApp/csx_locolens_webapp_callback.h"

class WebAppInterface 
{
public:
    void WebAppSendThread(GPSBuffer& buffer, SwitchBuffer& switch_buffer);

private:
    // data member
    double switch_trigger_flag; 
    // create an instance of the Callback class
    //Callback cb_instance;
};

#endif //_CSX_LOCOLENS_WEBAPP_H_
