#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <sstream>
#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <coav/coav.hh>

namespace 
{
    static const uint16_t DepthMin = 330;   //0.33 m
    static const uint16_t DepthMax = 15000; //15 m

    using MinMaxPairI = std::pair<uint16_t, uint16_t>;
    using MinMaxPairF = std::pair<float, float> ;
}

void LogObstacle(const Obstacle& o)
{
    std::cout << std::fixed << 
        "\tid " << std::setw(5) << 
o.id << 
        std::setprecision(3) << 
        " [R " << o.center.x << " m" << std::setprecision(0) << 
        " | h " << std::setw(3) << glm::degrees(o.center.z) << "° " << 
        " | v " << std::setw(3) << glm::degrees(o.center.y) << "° " << 
        " | x " << std::setw(4) << o.center_cartesian.x << " px " <<
        " | y " << std::setw(3) << o.center_cartesian.y << " px " <<
        "]" << std::endl;
}

template<typename T>
std::vector<T> GetRawBuffer(const rs2::video_frame& df)
{
   const size_t resSize = df.get_data_size() / sizeof(T);
   const T* first(reinterpret_cast<const T*>(df.get_data()));
   const T* last = first + resSize;

   std::vector<T> res(resSize, 0);
   std::copy(first, last, res.begin());
   return res;
}

std::pair<uint16_t, uint16_t> GetRawDepthBounds(const std::vector<uint16_t>& rawDepthVec_)
{
    std::vector<uint16_t> rawDepthVec = rawDepthVec_;
    auto itEndNonzero = std::remove(rawDepthVec.begin(), rawDepthVec.end(), 0);
    rawDepthVec.erase(itEndNonzero, rawDepthVec.end());

    const auto [minDepth, maxDepth] = std::minmax_element(rawDepthVec.begin(), rawDepthVec.end());
    return std::pair<uint16_t, uint16_t>(*minDepth, *maxDepth);
}

MinMaxPairI GetRawDepthBounds(const std::vector<uint16_t>& rawDepthVec, const MinMaxPairI& filter)
{
    std::vector<uint16_t> rawDepthVec_ = rawDepthVec;
    auto itEndFiltered = std::remove_if(rawDepthVec_.begin(), rawDepthVec_.end(), [&filter](uint16_t val){
        return (val <= filter.first) || (val > filter.second);
    });
    rawDepthVec_.erase(itEndFiltered, rawDepthVec_.end());

    const auto [itMinDepth, itMaxDepth] = std::minmax_element(rawDepthVec_.begin(), rawDepthVec_.end());
    return MinMaxPairI(*itMinDepth, *itMaxDepth);
}

MinMaxPairF GetDepthBounds(const rs2::depth_frame& frame, const MinMaxPairI& filter)
{
    const auto rawBounds = GetRawDepthBounds(GetRawBuffer<uint16_t>(frame), filter);
    return MinMaxPairF(
        rawBounds.first  * frame.get_units(),
        rawBounds.second * frame.get_units()
    );
}

std::string Timestamp(const std::string& format)
{
    const auto now = std::chrono::system_clock::now();
    const auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), format.c_str());
    return ss.str();
}

void SaveFrame(const rs2::video_frame& frame, const uint8_t* rawBuffer = nullptr)
{
    if (rawBuffer == nullptr)
    {
        rawBuffer = static_cast<const uint8_t*>(frame.get_data());
    }

    // Write images to disk
    std::stringstream filename;
    filename << "image_" << Timestamp("%F_%H%M%S") << "_" << frame.get_profile().stream_name() << ".png";
    stbi_write_png(filename.str().c_str(), frame.get_width(), frame.get_height(),
                   frame.get_bytes_per_pixel(), rawBuffer, frame.get_stride_in_bytes());
}

void DrawObstacles(std::vector<uint8_t>& imageBuf, const std::vector<Obstacle>& obstacles)
{
}

int main(int argc, char **argv)
{
    rs2::pipeline pipeline;
    pipeline.start();
    auto depth_camera = std::make_shared<RealSense2CamProxy>();
    auto detectorObstacle = std::make_shared<DepthImageObstacleDetector>();

    std::cout << "Press q to exit, Enter to capture next frame" << std::endl;

    while (std::cin.get() != 'q') 
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::depth_frame depthFrame = frames.get_depth_frame();
        rs2::video_frame colorFrame = frames.get_color_frame();
        const rs2::video_frame depthFrameColorized = rs2::colorizer().colorize(depthFrame); 

        const auto depthBounds = GetDepthBounds(depthFrame, MinMaxPairI(0, INT_MAX));
        const auto depthBoundsFilt = GetDepthBounds(depthFrame, MinMaxPairI(DepthMin, DepthMax));
        std::cout << Timestamp("%F_%H%M%S") << std::setprecision(3) << 
            " R ["        << depthBounds.first << "~" << depthBounds.second << "] m" << 
            " filtered [" << depthBoundsFilt.first << "~" << depthBoundsFilt.second << "] m" << 
            std::endl;

        depth_camera->set_depth_frame(&depthFrame);
        const auto depthData = depth_camera->read();
        const std::vector<Obstacle> obstacles = detectorObstacle->detect(depthData); 

        std::cout << "DepthImageObstacleDetector " << std::endl;
        for (const Obstacle& o: obstacles)
        {
            LogObstacle(o);       
        }

        std::vector<uint8_t> colorFrameBuf = GetRawBuffer<uint8_t>(colorFrame);
        DrawObstacles(colorFrameBuf, obstacles);

        SaveFrame(depthFrameColorized);
        SaveFrame(colorFrame, colorFrameBuf.data());
    }
}
