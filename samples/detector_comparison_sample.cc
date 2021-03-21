#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>

#include <coav/coav.hh>

void LogObstacle(const Obstacle& o)
{
    std::cout << std::fixed << 
        "\tid " << std::setw(5) << o.id << 
        std::setprecision(3) << 
        " [R " << o.center.x << " m" << std::setprecision(0) << 
        " | v " << std::setw(3) << glm::degrees(o.center.y) << "°" <<
        " | h " << std::setw(3) << glm::degrees(o.center.z) << "°" <<
        "]" << std::endl;
}

int main(int argc, char **argv)
{
    std::shared_ptr<DepthCamera> depth_camera = std::make_shared<RealSense2Camera>();

    static const double PolarHistStep = glm::radians(5.0); //i.e. 5 degrees in radians
    auto detectorObstacle = std::make_shared<DepthImageObstacleDetector>();
    auto detectorPolarHist = std::make_shared<DepthImagePolarHistDetector>(PolarHistStep);

    while (true) 
    {
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
