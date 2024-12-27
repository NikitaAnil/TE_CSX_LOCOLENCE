/**
* @file     csx_locolens_webapp_callbach.h
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

class  Callback  : public virtual mqtt::callback
{
public:
    explicit  Callback (SwitchBuffer &buf) : buffer_(buf) {}

    virtual void connected(const std::string &cause) = 0;

    virtual void connection_lost(const std::string &cause) = 0;

    virtual void message_arrived(mqtt::const_message_ptr msg) = 0;

private:
    // data member
    double switch_trigger_flag; 
    SwitchBuffer &buffer_;
};

#endif //_CSX_LOCOLENS_WEBAPP_H_
