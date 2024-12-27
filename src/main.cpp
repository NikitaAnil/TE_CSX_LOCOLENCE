#include "Radar/csx_locolens_radar.h"
#include "const_config.h"
#include "RTK_GPS/csx_locolens_rtk_gps.h"
#include "Velocity/csx_locolens_velocity.h"
#include "Camera/csx_locolens_camera.h"
#include "Processing_Module/csx_locolens_processing_module.h"
#include "WebApp/csx_locolens_webapp.h"
#include <iostream>
#include <thread>

#include "RTK_GPS/csx_locolence_rtk_gps_recv.h"
#include "WebApp/csx_locolens_switch_alert.h"

std::mutex data_mutex;
std::condition_variable cv_to_start;
bool start_Processing = false;


int main()
{

    GPSBuffer gps_buffer_front(10);
    GPSBuffer gps_buffer_rear(10);
    SwitchBuffer switch_buffer;
    RtkGpsRecv gps_recv_iface;
    RtkGpsInterface rtk_gps_iface;
    ntrip_caster_socket = gps_recv_iface.ConnectToNTRIPCaster(kNtripCasterHost,
                                      kNtripCasterPort,
                                      kUserName,
                                      kPassword,
                                      kMountPoint);

    std::thread front_datagen(&RtkGpsInterface::ReadGPSDataFront, &rtk_gps_iface, std::ref(gps_buffer_front), FRONT);
    std::thread rear_datagen(&RtkGpsInterface::ReadGPSDataRear, &rtk_gps_iface, std::ref(gps_buffer_rear), REAR);
    
    VelocityInterface velocity_iface;
    // Start the velocity calculation thread
    std::thread velocity_thread(&VelocityInterface::DirectionVelocityThread, 
                                          &velocity_iface, 
                                          std::ref(gps_buffer_front),
                                          std::ref(gps_buffer_rear));

    WebAppInterface web_app_iface;
    std::thread webapp_thread(&WebAppInterface::WebAppSendThread,
                              &web_app_iface,
                              std::ref(gps_buffer_front),
                              std::ref(switch_buffer));
    SwitchAlert switch_alert_iface;
    std::thread switch_alert_thread(&SwitchAlert::SwitchAlertThread,
                                    &switch_alert_iface,
                                    std::ref(gps_buffer_front),
                                    std::ref(switch_buffer));
    front_datagen.join();
    rear_datagen.join();
    velocity_thread.join();
    webapp_thread.join();
    switch_alert_thread.join();





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


    ProcessingModuleInterface processing_module_iface;
    // Start the processing module thread
    std::thread processing_module_thread(&ProcessingModuleInterface::ProcessModule, &processing_module_iface);
    // Start the final output thread
    std::thread final_output_thread_handle(&ProcessingModuleInterface::FinalOutput, &processing_module_iface);
    // Join all threads

    can_thread_front.join();
    can_thread_back.join();
    image_thread_short_front.join();
    image_thread_short_back.join();
    image_thread_long_front.join();
    image_thread_long_back.join();

    processing_module_thread.join();
    final_output_thread_handle.join();

    return 0;
}