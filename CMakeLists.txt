# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(LocoLens)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the include directory for header files
include_directories(${PROJECT_SOURCE_DIR}/include)

# Specify all source files
set(SOURCES
   src/main.cpp
   src/Processing_Module/csx_locolens_processing_module.cpp
   src/Camera/csx_locolens_camera.cpp
   src/Radar/csx_locolens_radar.cpp
   src/RTK_GPS/csx_locolens_rtk_gps.cpp
   src/Velocity/csx_locolens_velocity.cpp
   src/WebApp/csx_locolens_webapp.cpp
)

# Specify all header files (optional but helpful for IDEs)
set(HEADERS
   include/Processing_Module/csx_locolens_processing_module.h
   include/Camera/csx_locolens_camera.h
   include/Radar/csx_locolens_radar.h
   include/RTK_GPS/csx_locolens_rtk_gps.h
   include/Velocity/csx_locolens_velocity.h
   include/WebApp/csx_locolens_webapp.h
   include/ring_buffer.h
)

# Add the executable
add_executable(LocoLens ${SOURCES} ${HEADERS})

# Link pthread
target_link_libraries(LocoLens PRIVATE pthread)
