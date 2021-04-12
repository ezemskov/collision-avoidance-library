#include <cmath>
#include <memory>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <coav/coav.hh>

namespace 
{
    namespace ObstacleLimits
    {
        static const size_t WidthPx = 300;
        static const size_t HeightPx = 300;
        static const uint16_t DepthUnits = 500;
        static const size_t PointsQnty = 200000;
    };
 
    static const uint16_t DepthMin = 330;   //0.33 m
    static const uint16_t DepthMax = 15000; //15 m

    static double DepthScale = 1.0;    //ugly

    using MinMaxPairI = std::pair<uint16_t, uint16_t>;
    using MinMaxPairF = std::pair<float, float>;
    using MinMaxPairD = std::pair<double, double>;
}

template<typename GetDimFuncT>
MinMaxPairD Bounds(const Obstacle& o, GetDimFuncT getDim)
{
    if (o.points.empty())
    {
        return MinMaxPairD(getDim(o.center), getDim(o.center)); 
    }

    const auto [itMin, itMax] = std::minmax_element(o.points.begin(), o.points.end(), [&getDim](const glm::dvec3& pA, const glm::dvec3& pB) -> bool {
        return getDim(pA) < getDim(pB);
    });

    return MinMaxPairD(getDim(*itMin), getDim(*itMax));
}

double Delta(const MinMaxPairD& bounds)
{
    return bounds.second - bounds.first;
} 

void LogObstacle(const Obstacle& o, std::ostream& os)
{
    const auto xBounds = Bounds(o, [](const glm::dvec3& point){ return point.z; });
    const auto yBounds = Bounds(o, [](const glm::dvec3& point){ return point.y; });
    const auto rBounds = Bounds(o, [](const glm::dvec3& point){ return point.x * DepthScale; });

    os << std::fixed << 
        "\tid " << std::setw(5) << o.id << std::setprecision(3) << 
        " R <" << o.center.x <<   "> [" << rBounds.first << "~" << rBounds.second << "] m" << std::setprecision(0) << 
        " | x <" << std::setw(4) << o.center_cartesian.z << "> [" << xBounds.first << "~" << xBounds.second << "] px " <<
        " | y <" << std::setw(3) << o.center_cartesian.y << "> [" << std::setw(3) << yBounds.first << "~" << std::setw(3) << yBounds.second << "] px " <<
        " | w " << std::setw(3) << Delta(xBounds) << " h " << std::setw(3) << Delta(yBounds) << 
        " | qnty " << std::setw(3) << o.points.size() <<
        std::endl;
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

MinMaxPairI GetRawDepthBounds(const std::vector<uint16_t>& rawDepthVec_)
{
    std::vector<uint16_t> rawDepthVec = rawDepthVec_;
    auto itEndNonzero = std::remove(rawDepthVec.begin(), rawDepthVec.end(), 0);
    rawDepthVec.erase(itEndNonzero, rawDepthVec.end());

    const auto [minDepth, maxDepth] = std::minmax_element(rawDepthVec.begin(), rawDepthVec.end());
    return MinMaxPairI(*minDepth, *maxDepth);
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

//Remove obstacles that contain points further/deeper than DepthMax
void FilterObstales(std::vector<Obstacle>& obstacles)
{
    auto itEndFiltered = std::remove_if(obstacles.begin(), obstacles.end(), [](const Obstacle& o){
        //any point is deeper than DepthMax
        const auto rBounds = Bounds(o, [](const glm::dvec3& point){ return point.x; });
        return (rBounds.second > DepthMax);
    });

    itEndFiltered = std::remove_if(obstacles.begin(), itEndFiltered, [](const Obstacle& o){
        //too many points
        return (o.points.size() > ObstacleLimits::PointsQnty);
    });

    itEndFiltered = std::remove_if(obstacles.begin(), itEndFiltered, [](const Obstacle& o){
        const auto xBounds = Bounds(o, [](const glm::dvec3& point){ return point.z; });
        const auto yBounds = Bounds(o, [](const glm::dvec3& point){ return point.y; });

        //too big
        return (Delta(xBounds) > ObstacleLimits::WidthPx) || 
               (Delta(yBounds) > ObstacleLimits::HeightPx);
    });

    itEndFiltered = std::remove_if(obstacles.begin(), itEndFiltered, [](const Obstacle& o){
        //too deep
        const auto rBounds = Bounds(o, [](const glm::dvec3& point){ return point.x; });
        return Delta(rBounds) > ObstacleLimits::DepthUnits;
    });

    obstacles.erase(itEndFiltered, obstacles.end());
}

std::string Timestamp(const std::string& format)
{
    const auto now = std::chrono::system_clock::now();
    const auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), format.c_str());
    return ss.str();
}

void SaveFrame(const rs2::video_frame& frame, const uint8_t* rawBuffer, const std::string& filename)
{
    if (rawBuffer == nullptr)
    {
        rawBuffer = static_cast<const uint8_t*>(frame.get_data());
    }

    // Write images to disk
    stbi_write_png(filename.c_str(), frame.get_width(), frame.get_height(),
                   frame.get_bytes_per_pixel(), rawBuffer, frame.get_stride_in_bytes());
}

void DrawObstacles(std::vector<uint8_t>& imageBuf, const std::vector<Obstacle>& obstacles)
{
    //todo : get from rs2::video_frame
    static const size_t Width = 848;
    //static const size_t Height = 480;
    static const size_t BPP = 3;

    for (const Obstacle& o: obstacles)
    {
        for (const glm::dvec3& point : o.points)
        {
            const size_t idx = (point.z + (point.y * Width)) * BPP;
            imageBuf[idx] = 0xFF;
            imageBuf[idx+1] = 0xFF;
            imageBuf[idx+2] = 0xFF;
        }
    }
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
        DepthScale = depthFrame.get_units();

        depth_camera->set_depth_frame(&depthFrame);
        const auto depthData = depth_camera->read();

        //Detect obstacles
        std::vector<Obstacle> obstacles = detectorObstacle->detect(depthData); 
        auto filteredObstacles = obstacles;
        FilterObstales(filteredObstacles);

        //Format log message
        std::stringstream logSS;
        const auto depthBounds = GetDepthBounds(depthFrame, MinMaxPairI(0, INT_MAX));
        const auto depthBoundsFilt = GetDepthBounds(depthFrame, MinMaxPairI(DepthMin, DepthMax));
        logSS << std::setprecision(3) << 
            "R ["        << depthBounds.first << "~" << depthBounds.second << "] m" << 
            " filtered [" << depthBoundsFilt.first << "~" << depthBoundsFilt.second << "] m" << 
            std::endl;

        logSS << "All obstacles " << std::endl;
        for (const Obstacle& o: obstacles)
        {
            LogObstacle(o, logSS);       
        }

        logSS << "Filtered obstacles " << std::endl;
        for (const Obstacle& o: filteredObstacles)
        {
            LogObstacle(o, logSS);       
        }

        //Log to console
        std::cout << Timestamp("%F_%H%M%S") << std::endl << logSS.rdbuf(); 

        //Log to file
        const std::string filenamePrefix  = "image_" + Timestamp("%F_%H%M%S"); 
        std::fstream logFS("./" + filenamePrefix + ".log", std::ios::out);
        logFS << logSS.str();
        logFS.flush();
        logFS.close();

        //Draw obstacles onto depth image
        std::vector<uint8_t> depthFrameBuf = GetRawBuffer<uint8_t>(depthFrameColorized);
        DrawObstacles(depthFrameBuf, filteredObstacles);

        //Save images
        SaveFrame(depthFrameColorized, depthFrameBuf.data(), filenamePrefix + "_depth_obst.png");
        SaveFrame(depthFrameColorized, nullptr,              filenamePrefix + "_depth.png");
        SaveFrame(colorFrame,          nullptr,              filenamePrefix + "_color.png");
    }
}
