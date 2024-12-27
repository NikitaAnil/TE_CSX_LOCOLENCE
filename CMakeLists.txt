# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(LocoLens)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(PAHO_MQTT_INCLUDE_DIR /usr/local/mqtt/paho-cpp/include)
set(PAHO_MQTT_LIBRARY_DIR /usr/local/mqtt/paho-cpp/lib)


# Add the include directory for header files
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PAHO_MQTT_INCLUDE_DIR})


find_library(PAHO_MQTTPP3_LIBRARY NAMES paho-mqttpp3 HINTS ${PAHO_MQTT_LIBRARY_DIR})
find_library(PAHO_MQTT3C_LIBRARY NAMES paho-mqtt3c HINTS ${PAHO_MQTT_LIBRARY_DIR})

# Specify all source files
set(SOURCES
   src/main.cpp
   src/Processing_Module/csx_locolens_processing_module.cpp
   src/Camera/csx_locolens_camera.cpp
   src/Radar/csx_locolens_radar.cpp
   src/RTK_GPS/csx_locolens_rtk_gps.cpp
   src/RTK_GPS/csx_locolence_rtk_gps_recv.cpp
   src/Velocity/csx_locolens_velocity.cpp
   src/Velocity/csx_locolens_distance_azimuth.cpp
   src/WebApp/csx_locolens_webapp.cpp
   src/WebApp/csx_locolens_switch_alert.cpp
   src/WebApp/csx_locolens_webapp_callback.cpp
)

# Specify all header files (optional but helpful for IDEs)
set(HEADERS
   include/Processing_Module/csx_locolens_processing_module.h
   include/Camera/csx_locolens_camera.h
   include/Radar/csx_locolens_radar.h
   include/RTK_GPS/csx_locolens_rtk_gps.h
   include/RTK_GPS/csx_locolence_rtk_gps_recv.h
   include/Velocity/csx_locolens_velocity.h
   include/Velocity/csx_locolens_distance_azimuth.h
   include/WebApp/csx_locolens_webapp.h
   include/WebApp/csx_locolens_switch_alert.h
   include/WebApp/csx_locolens_webapp_callback.h
   include/const_config.h
)

# Add the executable
add_executable(LocoLens ${SOURCES} ${HEADERS})

# Link pthread
target_link_libraries(LocoLens PRIVATE pthread ${PAHO_MQTTPP3_LIBRARY} ${PAHO_MQTT3C_LIBRARY})

