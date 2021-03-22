#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "sensors/Sensors.hh"

namespace rs2
{
    class depth_frame;
}

class RealSense2CamProxy: public DepthCamera
{
public:
    RealSense2CamProxy();

    void set_depth_frame(rs2::depth_frame* frame_) { frame = frame_; } 
    std::vector<uint16_t> &get_depth_buffer() override;

private:
    std::vector<uint16_t> depth_buffer;
    rs2::depth_frame* frame = nullptr;
};
