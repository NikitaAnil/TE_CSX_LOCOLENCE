#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>
#include <deque>

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


#endif