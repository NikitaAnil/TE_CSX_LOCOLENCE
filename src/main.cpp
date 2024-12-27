#include "Radar/csx_locolens_radar.h"
#include "ring_buffer.h"
#include "RTK_GPS/csx_locolens_rtk_gps.h"
#include "Velocity/csx_locolens_velocity.h"
#include "Camera/csx_locolens_camera.h"
#include "Processing_Module/csx_locolens_processing_module.h"
#include "WebApp/csx_locolens_webapp.h"
#include <iostream>
#include <thread>

std::mutex data_mutex;
std::condition_variable cv_to_start;
bool start_Processing = false;


int main()
{
    RtkGpsInterface rtk_gps_iface;
    RingBufferGPS gpsBuffer(10); // Create a ring buffer with capacity 10
    // Start the RTK GPS threads
    std::thread rtk_gps_front_thread(&RtkGpsInterface::RtkGpsFront, &rtk_gps_iface, std::ref(gpsBuffer));
    std::thread rtk_gps_back_thread(&RtkGpsInterface::RtkGpsBack, &rtk_gps_iface, std::ref(gpsBuffer));
    
    VelocityInterface velocity_iface;
    // Start the velocity calculation thread
    std::thread velocity_thread_handle(&VelocityInterface::CalculateVelocity, &velocity_iface, std::ref(gpsBuffer));

    // Below time slotfor thread is to handle the time lapse from RTK float to Fixed mode
    std::this_thread::sleep_for(std::chrono::seconds(10));
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        start_Processing = true;
    }
    cv_to_start.notify_all();

    // Start threads for CAN and image data collection
    RingBuffer<CANData> can_buffer(3);
    RingBuffer<ImageData> image_buffer(3);

    RadarInterface radar_cam_iface;
   
    std::thread can_thread_front(&RadarInterface::CollectCANDatafront, &radar_cam_iface, std::ref(can_buffer));
    std::thread can_thread_back(&RadarInterface::CollectCANDataBack,  &radar_cam_iface, std::ref(can_buffer));
    
    CameraInterface camera_iface;
    std::thread image_thread_short_front(&CameraInterface::FrontShortRangeCam,  &camera_iface, std::ref(image_buffer));// one more paramete for camera url/path
    std::thread image_thread_short_back(&CameraInterface::FrontLongRangeCam,  &camera_iface, std::ref(image_buffer));// one more paramete for camera url/path
    std::thread image_thread_long_front(&CameraInterface::BackShortRangeCam,  &camera_iface, std::ref(image_buffer));// one more paramete for camera url/path
    std::thread image_thread_long_back(&CameraInterface::BackLongRangeCam,  &camera_iface, std::ref(image_buffer));// one more paramete for camera url/path

    //std::thread change_mode_thread(ChangeMode);
    WebAppInterface web_app_iface;
    // Start the webApp server thread
    std::thread web_app_server_thread(&WebAppInterface::WebAppThread, &web_app_iface);

    ProcessingModuleInterface processing_module_iface;
    // Start the processing module thread
    std::thread processing_module_thread(&ProcessingModuleInterface::ProcessModule, &processing_module_iface);
    // Start the final output thread
    std::thread final_output_thread_handle(&ProcessingModuleInterface::FinalOutput, &processing_module_iface);
    // Join all threads
    rtk_gps_front_thread.join();
    rtk_gps_back_thread.join();
    velocity_thread_handle.join();
    web_app_server_thread.join();
    processing_module_thread.join();
    final_output_thread_handle.join();

    can_thread_front.join();
    can_thread_back.join();
    image_thread_short_front.join();
    image_thread_short_back.join();
    image_thread_long_front.join();
    image_thread_long_back.join();

    return 0;
}