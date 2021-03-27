#include "sensors/RealSense2CamProxy.hh"

#include <librealsense2/rs.hpp>

#include <iostream>
#include <limits>

#include <glm/glm.hpp>


RealSense2CamProxy::RealSense2CamProxy() try
{
    width = 848;
    height = 480;

    //todo
    hfov = glm::radians(87.0);
    vfov = glm::radians(58.0);
    scale = std::numeric_limits<double>::quiet_NaN();  //will be set in first set_depth_frame()

    depth_buffer.resize(width * height);
}
catch(const rs2::error &e)
{
    std::cerr << "[RealSense2CamProxy] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;
}

void RealSense2CamProxy::set_depth_frame(rs2::depth_frame* frame_)
{ 
    frame = frame_; 
    if (frame != nullptr)
    {
        scale = frame->get_units();
    }
} 

std::vector<uint16_t> &RealSense2CamProxy::get_depth_buffer() try
{
    if (frame == nullptr)
    {
        return depth_buffer;
    }

    const uint16_t *frameData = reinterpret_cast<const uint16_t *>(frame->get_data());

    for (size_t i = 0; i < width * height; i++) //todo : std::copy
        depth_buffer[i] = frameData[i];

    return depth_buffer;
}
catch(const rs2::error &e)
{
    std::cerr << "[RealSense2CamProxy] Error: " << e.get_failed_function().c_str()
        << e.get_failed_args().c_str() << std::endl;

    return depth_buffer;
}
