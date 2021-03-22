#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>

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
