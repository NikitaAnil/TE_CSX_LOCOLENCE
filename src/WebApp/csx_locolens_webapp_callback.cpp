/**
* @file     csx_locolens_webapp.cpp
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
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

// local header file
#include "WebApp/csx_locolens_webapp_callback.h"


/**
* @brief -   This function is invoked when the MQTT client establishes a connection
*            with the MQTT broker. It logs a message indicating that the connection
*            was successful along with any associated information provided.
* @param    cause: A string providing information about the cause of the connection.
* @return   void
*/
void Callback::connected(const std::string &cause)
{
    std::cout << "Connected to MQTT broker: " << cause << std::endl;
}


/**
* @brief -   This function is invoked when the MQTT client loses connection to
*            the broker. It logs the cause of the disconnection to the console.
* @param    cause: A string describing the reason for the connection loss.
* @return   void
*/
void Callback::connection_lost(const std::string &cause)
{
    std::cerr << "Connection lost: " << cause << std::endl;
}

/**
* @brief -   This function is called when a message arrives on a subscribed topic.
*            It converts the received message to a JSON object, parses the JSON and
*            stores the switch data provided to the switch buffer.
* @param    msg: A shared pointer to the received MQTT message.
* @return   void
*/
void Callback::message_arrived(mqtt::const_message_ptr msg)
{
    std::cout << "Message received on topic: " << msg->get_topic()
                << " -> " << msg->to_string() << std::endl;
    nlohmann::json switch_data = nlohmann::json::parse(msg->to_string());
    std::vector<std::pair<double, double>> switch_coordinates;
    for (size_t i = 0; i < switch_data["switches"].size(); i++)
    {
        double lat = switch_data["switches"][i]["geometry"]
                                ["coordinates"][1].get<double>();
        double lon = switch_data["switches"][i]["geometry"]
                                ["coordinates"][0].get<double>();
        switch_coordinates.emplace_back(lat, lon);
    }
    buffer_.Push(switch_coordinates);
}