#include "sensors/RealSense2Camera.hh"

#include <iostream>
#include <limits>

#include <glm/glm.hpp>


RealSense2Camera::RealSense2Camera() try
{
    this->width = 848;
    this->height = 480;

    //todo
    this->hfov = glm::radians(87.0);
    this->vfov = glm::radians(58.0);
    this->scale = std::numeric_limits<double>::quiet_NaN();  //will be set in first get_depth_buffer()

    pipeline.start();

    this->depth_buffer.resize(this->width * this->height);
}
catch(const rs2::error &e)
{
    std::cerr << "[RealSense2Camera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;
}

std::vector<uint16_t> &RealSense2Camera::get_depth_buffer() try
{
    //todo : check if started
    if (false) {
        std::cerr << "[RealSense2Camera] Error: No device available." << std::endl;
        return this->depth_buffer;
    }

    rs2::frameset frames = this->pipeline.wait_for_frames();
    rs2::depth_frame frame = frames.get_depth_frame();

    this->scale = frame.get_units();

    //todo : query from pipeline/somewhere ?
    const auto intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    std::cout << "dimensions [" << intr.width << "x" << intr.height << "]" << std::endl; 

    const uint16_t *frameData = reinterpret_cast<const uint16_t *>(frame.get_data());

    std::lock_guard<std::mutex> locker(depth_buffer_mtx);
    for (size_t i = 0; i < this->width * this->height; i++) //todo : std::copy
        this->depth_buffer[i] = frameData[i];

    return this->depth_buffer;
}
catch(const rs2::error &e)
{
    std::cerr << "[RealSenseCamera] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;

    return this->depth_buffer;
}
