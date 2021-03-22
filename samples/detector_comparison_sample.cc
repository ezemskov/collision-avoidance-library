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

std::vector<uint16_t> GetRawDepths(const rs2::depth_frame& df)
{
   const size_t pointQnty = df.get_data_size() / sizeof(uint16_t);
   const uint16_t* first(reinterpret_cast<const uint16_t*>(df.get_data()));
   const uint16_t* last = first + pointQnty;

   std::vector<uint16_t> res(pointQnty, 0);
   std::copy(first, last, res.begin());
   return res;
}

std::pair<uint16_t, uint16_t> GetRawDepthBounds(const std::vector<uint16_t>& rawDepthVec_)
{
    std::vector<uint16_t> rawDepthVec = rawDepthVec_;
    auto itEndNonzero = std::remove(rawDepthVec.begin(), rawDepthVec.end(), 0);
    rawDepthVec.erase(itEndNonzero, rawDepthVec.end());

    const auto [minDepthRaw, maxDepthRaw] = std::minmax_element(rawDepthVec.begin(), rawDepthVec.end());
    return std::pair<uint16_t, uint16_t>(*minDepthRaw, *maxDepthRaw);
}

std::string Timestamp(const std::string& format)
{
    const auto now = std::chrono::system_clock::now();
    const auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), format.c_str());
    return ss.str();
}

void SaveFrame(const rs2::video_frame& frame_)
{
    const bool isDepth = frame_.is<rs2::depth_frame>();
    //rs2::colorizer colorizer;
    const rs2::video_frame frame = isDepth ? rs2::colorizer().colorize(frame_) : frame_; 

    // Write images to disk
    std::stringstream filename;
    filename << "image_" << Timestamp("%H%M%S") <<  frame.get_profile().stream_name() << ".png";
    stbi_write_png(filename.str().c_str(), frame.get_width(), frame.get_height(),
                   frame.get_bytes_per_pixel(), frame.get_data(), frame.get_stride_in_bytes());
}


int main(int argc, char **argv)
{
    rs2::pipeline pipeline;
    pipeline.start();
    auto depth_camera = std::make_shared<RealSense2CamProxy>();

    static const double PolarHistStep = glm::radians(5.0); //i.e. 5 degrees in radians
    auto detectorObstacle = std::make_shared<DepthImageObstacleDetector>();
    auto detectorPolarHist = std::make_shared<DepthImagePolarHistDetector>(PolarHistStep);

    std::cout << "Press q to exit, Enter to capture next frame" << std::endl;

    while (std::cin.get() != 'q') 
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::depth_frame depthFrame = frames.get_depth_frame();

        SaveFrame(depthFrame);
        SaveFrame(frames.get_color_frame());

        const auto depthBounds = GetRawDepthBounds(GetRawDepths(depthFrame));
        const float minDepth = depthBounds.first * depthFrame.get_units(); 
        const float maxDepth = depthBounds.second * depthFrame.get_units(); 
        std::cout << Timestamp("%T") << std::setprecision(3) << " R bounds [" << minDepth << "~" << maxDepth << "] m" << std::endl;

        depth_camera->set_depth_frame(&depthFrame);
        const auto depthData = depth_camera->read();

        std::cout << "DepthImageObstacleDetector " << std::endl;
        for (const Obstacle& o: detectorObstacle->detect(depthData))
        {
            LogObstacle(o);       
        }

        std::cout << "DepthImagePolarHistDetector " << std::endl;
        for (const Obstacle& o: detectorPolarHist->detect(depthData))
        {
            LogObstacle(o);       
        }
    }
}
