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
#include "WebApp/csx_locolens_webapp.h"
#include "mqtt/async_client.h"


/**
* @brief -   This function sets up the MQTT client for communication with the webapp.
*            The client is subscribed to topic 'machine/switches' and asynchronously
*            waits for messages. The function then repeatedly sends the current location
*            of the locomotive to the webapp.
* @param     buffer: A reference to the `GPSBuffer` object to obtain the current
*                    location of the locomotive.
* @param     switch_buffer: A reference to the `SwitchBuffer` object to store the
*                           coordinates of the switches.
* @return   void
*/
void WebAppInterface::WebAppSendThread(GPSBuffer& buffer, SwitchBuffer& switch_buffer)
{
    mqtt::async_client client(kServerAddress, kClientId);
    //Callback cb_instance(switch_buffer);
    //client.set_callback(cb_instance);

    try 
    {
        // Connect to the MQTT broker
        mqtt::connect_options connOpts;
        connOpts.set_clean_session(true);
        client.connect(connOpts)->wait();

        // Subscribe to a topic
        client.subscribe(kTopicSubscribe, kQos)->wait();
        std::cout << "Subscribed to topic: " << kTopicSubscribe << std::endl;
        std::cout << "Sending location data to webserver..." << std::endl;
        while (true) 
        {
            if (direction.load() != STATIONARY) 
            {
                auto vehicle_coordinates = std::get<1>(buffer.Pop());
                nlohmann::json vehicle_status = { { "coordinates",
                    { vehicle_coordinates.second, vehicle_coordinates.first } } };
                std::string message = vehicle_status.dump();
                // Publish the message
                client.publish(kTopicPublish, message, kQos, false)->wait();
                std::cout << "Published: " << message << " to topic: "
                          << kTopicPublish << std::endl;

                // Sleep for 30 seconds - change according to webapp capability
                std::this_thread::sleep_for(std::chrono::seconds(30));
            }
        }
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "MQTT Error: " << e.what() << std::endl;
        return;
    }

    try 
    {
        client.disconnect()->wait();
        std::cout << "Disconnected from MQTT broker." << std::endl;
    } 
    catch (const mqtt::exception& e) 
    {
        std::cerr << "MQTT Error during disconnect: " << e.what() << std::endl;
    }
}


