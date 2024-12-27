/**
* @file     const_config.h
* @author   Nikita Anil
* @brief    This file contains the configuration values for RTK GPS, including the
*           serial port parameters, and the NTRIP caster configuration. It also contains
*           the configuration parameters of the MQTT client for the webserver, the
*           various constants used for distance and velocity calculations, and the
*           threshold parameters for velocity and switch distance.
* @copyright  Tata Elxsi (c) Copyright 2024
*/

#ifndef _CONST_CONFIG_H_
#define _CONST_CONFIG_H_

// system header file
#include <iostream>
#include <thread>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <tuple>
#include <utility>
#include <vector>
#include <deque>


enum RtkModule
{
    FRONT,
    REAR
};

enum Direction : unsigned int
{
    FORWARD,
    REVERSE,
    STATIONARY
};

/**
* @brief - This file defines the 'GPSBuffer' class, which is responsible for
*          receiving the location information through serial connection and storing it 
*          in a vector. It includes functions to add data to the vector once new data 
*          is received, and to return the location information for processing.
*/
class GPSBuffer
{
public:
    explicit GPSBuffer(size_t size) : buffer_(size), head_(0), tail_(0),
                             capacity_(size), current_size_(0) {}

    /**
    * @brief - This function takes the timestamp and coordinates to create a tuple,
    *          which is then pushed to the circular buffer. The function also updates
    *          the head and tail of the buffer according to the data available. It then
    *          notifies any threads that are waiting for switch data. Thread safety is
    *          ensured using a mutex.
    * @param   timestamp: The timestamp when the GPS data was obtained.
    * @param   coord: The location data that has been obtained from the module.
    * @return  void
    */
    void Push(const double timestamp, std::pair<double, double> coord)
    {
        std::unique_lock<std::mutex> lock(gps_mutex_);
        buffer_[tail_] = std::make_tuple(timestamp, coord);
        tail_ = (tail_ + 1) % capacity_;

        if (current_size_ < capacity_)
        {
            current_size_++;
        }
        else
        {
            head_ = (head_ + 1) % capacity_;
        }
        gps_cv_.notify_one();
    }

    /**
    * @brief - This function waits till data is available in the GPS buffer, then
    *          gets the next entry in the buffer. It also updates the head of the 
    *          circular buffer. Thread safety is ensured using a mutex.
    * @return  A tuple of the timestamp and the GPS coordinates.
    */
    std::tuple<long double, std::pair<double, double>> Pop()
    {
        std::unique_lock<std::mutex> lock(gps_mutex_);
        gps_cv_.wait(lock, [this]()
                     { return current_size_ > 0; });

        if (buffer_.empty())
        {
            throw std::runtime_error("Buffer is empty");
        }

        auto coord = buffer_[head_];
        head_ = (head_ + 1) % capacity_;
        current_size_--;
        return coord;
    }

private:
    std::vector<std::tuple<long double, std::pair<double, double>>> buffer_;
    size_t head_;
    size_t tail_;
    size_t capacity_;
    size_t current_size_;
    std::mutex gps_mutex_;
    std::condition_variable gps_cv_;
};




class SwitchBuffer
{
public:
    explicit SwitchBuffer() : current_size_(0), new_data_available_(false) {}
    /**
    * @brief - This function checks if the switch data provided by the webapp is
    *          different from what was provided in the previous update, sets the flag
    *          to true and replaces the switch buffer with the new data. It then
    *          notifies any threads that are waiting for switch data. Thread safety is
    *          ensured using a mutex.
    * @param   updated_buffer: A constant reference to the updated switch
    *                       data, stored as a vector.
    * @return  void
    */
    void Push(const std::vector<std::pair<double, double>> &updated_buffer)
    {
        std::unique_lock<std::mutex> lock(buffer_mutex_);

        // Only update the buffer, if the switches have changed
        if (updated_buffer != buffer_)
        {
            buffer_ = updated_buffer; // Replace the entire buffer
            new_data_available_ = true;
            std::cout << "Switch buffer updated" << std::endl;
        }
        else
        {
            std::cout << "No new data, buffer not updated" << std::endl;
        }

        current_size_ = buffer_.size();
        buffer_cv_.notify_one(); // Notify any threads waiting for switch data
    }

    /**
    * @brief - This function ensures that data is available in the switch buffer, then
    *          returns a copy of the switch buffer, and sets the `new_data_available_`
    *          flag to false. Thread safety is ensured using a mutex.
    * @param   NA
    * @return  void
    */
    std::vector<std::pair<double, double>> Pop()
    {
        std::unique_lock<std::mutex> lock(buffer_mutex_);

        // Wait until new data is available in the buffer
        buffer_cv_.wait(lock, [this]()
                        { return current_size_ > 0; });

        // If buffer is still empty after condition variable notification
        if (buffer_.empty())
        {
            throw std::runtime_error("Buffer is empty");
        }

        // Return a copy of the buffer
        std::vector<std::pair<double, double>> copy = buffer_;
        new_data_available_ = false;
        return copy;
    }

    /**
    * @brief - This function returns the `new_data_available_` flag, which indicates
    *          whether the switch data has been replaced by the webapp or not. Thread
    *          safety is ensured using a mutex.
    * @param   NA
    * @return  void
    */
    bool CheckForNewData()
    {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        return new_data_available_;
    }

private:
    std::vector<std::pair<double, double>> buffer_;
    size_t current_size_;
    std::mutex buffer_mutex_;
    std::condition_variable buffer_cv_;
    bool new_data_available_;
};



// GPS module Configuration
constexpr const char *kRtkportFront = "/dev/ttyACM0"; // Front RTK Port
constexpr const char *kRtkportRear = "/dev/ttyACM1";  // Rear RTK Port

// NTRIP Configuration
constexpr const char *kNtripCasterHost = "caster.emlid.com";
const unsigned int kNtripCasterPort = 2101;
constexpr const char *kMountPoint = "MP17162";
constexpr const char *kUserName = "u45675";
constexpr const char *kPassword = "496frn";

// Earth's semi-major axis (in meters)
constexpr double kSemiMajorAxis = 6378137.0;
// Earth's flattening factor
constexpr double kFlattening = (1.0 / 298.257223563);
// Earth's semi-minor axis (in meters)
constexpr double kSemiMinorAxis = 6356752.314245;
// Mathematical constant Pi
constexpr double kPi = 3.14159265358979323846;

constexpr double kVelocityThreshold = 5.0;         // Velocity threshold (m/s)
constexpr double kSwitchDistanceThreshold = 500.0; // Distance threshold (m)

// MQTT configuration
// server address
constexpr const char *kServerAddress = "tcp://localhost:1883";
// client identifier
constexpr const char *kClientId = "mqtt_cpp_client";
// subscription topic  
constexpr const char *kTopicSubscribe = "machine/switches";
// publication topic
constexpr const char *kTopicPublish = "machine/status";
// Quality of Service (QoS) level
constexpr int kQos = 1;




// Shared socket descriptor for NTRIP Caster connection
extern int ntrip_caster_socket;
// Flag to signal direction of movement - to be used by sensor selection thread
extern std::atomic<Direction> direction;
// Flag to signal whether velocity is above threshold 
// to be used by camera selection thread
extern std::atomic<bool> velocity_above_threshold;
// Flag to trigger the turnout switch detection thread
extern std::atomic<bool> switch_distance_below_threshold;









// Structures
struct CANData
{
    long double timestamp;
    std::vector<std::string> can_frame;
};

struct ImageData
{
    long double timestamp;
    //cv::Mat image;
};

// Ring buffer implementation
template <typename T>
struct RingBufferEntry
{
    T data;
    long double produced_time;
    long double consumed_time;
};

template <typename T>
class RingBuffer
{
    std::deque<RingBufferEntry<T>> buffer_;
    std::mutex mutex_;
    int max_size_;

    public:
    RingBuffer(int max_size) : max_size_(max_size) {}
    void AddData(const T &data);
    // Add all other functions related to Ring buffer of Can data and Image data

};

class RingBufferGPS
{
    public:
    using GPSData = std::tuple<std::string, long double, std::vector<double>>;

    RingBufferGPS(size_t capacity) : capacity(capacity), head(0), full(false)
    {
        buffer.resize(capacity);
    }
    // Add other functions related to GPS Ring buffer
    private:
    std::vector<GPSData> buffer;
    size_t head, capacity;
    bool full;

    mutable std::mutex mtx;                // Mutex for thread safety
    std::condition_variable dataAvailable; // Notify when new data is added
};

#endif //_LOCOLENS_CONST_CONFIG_H_
