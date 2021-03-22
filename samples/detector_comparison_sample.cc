#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>

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

int main(int argc, char **argv)
{
    rs2::pipeline pipeline;
    pipeline.start();
    auto depth_camera = std::make_shared<RealSense2CamProxy>();

    static const double PolarHistStep = glm::radians(5.0); //i.e. 5 degrees in radians
    auto detectorObstacle = std::make_shared<DepthImageObstacleDetector>();
    auto detectorPolarHist = std::make_shared<DepthImagePolarHistDetector>(PolarHistStep);

    while (true) 
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        rs2::depth_frame frame = frames.get_depth_frame();

        const auto depthBounds = GetRawDepthBounds(GetRawDepths(frame));
        const float minDepth = depthBounds.first * frame.get_units(); 
        const float maxDepth = depthBounds.second * frame.get_units(); 
        std::cout << std::setprecision(3) << "R bounds [" << minDepth << "~" << maxDepth << "] m" << std::endl;

        depth_camera->set_depth_frame(&frame);
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
