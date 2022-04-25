// Include the ROS library
    #include <ros/ros.h>

    // Include pcl
    #include <pcl_conversions/pcl_conversions.h>
    #include <pcl/point_cloud.h>
    #include <pcl/point_types.h>
    #include <cmath>
    #include <opencv2/opencv.hpp>
    #include <functional>
    #include <image_transport/image_transport.h>
    // Include PointCloud2 message
    #include <sensor_msgs/PointCloud2.h>
    #include <cv_bridge/cv_bridge.h>

    #include <cassert>

    #include <string>
    #include <utility>


    #include <algorithm>
    #include <vector>
    #include <numeric>
    #include <iostream>
    #include "../include/pointcloud/types.h"
    #include "../include/pointcloud/configuration.h"
    static const std::string Point_Topic = "/camera/depth/color/points";
    const float sensorMinimumRange = 0.4;
    using Coordinate_t = double;
    using PointIndex_t = uint32_t;
    using Height_t = uint16_t;

    using namespace cv;
    using namespace pcl;
    using namespace std;
    using NumPoints_t = uint32_t;
    using Histogram_t = vector<NumPoints_t>;
    using Peaks_t = vector<Height_t>;

    //PointsHt_t calcHeights(const pcl::PointXYZ &points);

    struct PointHt
    {
        PointIndex_t pointIdx;
        Height_t height;
    };
    using PointsHt_t = vector<PointHt>;


    
    struct Plateau
    {
    Height_t height;
    PointsHt_t plateauPoints;
        //Quadrilateral_t quadriWorld2D; // only x, y
    bool valid = false;
    };
        

    using Plateaus_t = vector<Plateau>;
    struct ProcessingConfiguration : Configuration
    {
    const Coordinate_t heightIntervalReciprocal = 1.0 / heightInterval;
 
    };
    const ProcessingConfiguration __processingConfiguration;
    // Topics
 
    
 // histogram index

 class HeightsHistogram
{
    public:
  static void calcHistogram(const PointsHt_t &points, Histogram_t &hist, Peaks_t &peaks)
  {
    const auto &config = __processingConfiguration;
    const auto zrange = config.measuringRange.z;

    hist = calcHist(points, zrange.max - zrange.min, config.heightIntervalReciprocal);
    peaks = detectPeaks(hist);
  }

    private:
  static Histogram_t calcHist(const PointsHt_t &points, Coordinate_t maxRelHeight, Coordinate_t intervalReciprocal)
  {
    Histogram_t hist(static_cast<size_t>(maxRelHeight * intervalReciprocal) + 1);

        for(auto i : points){
            ++hist[i.height];
        }
    return hist;
  }

  static Peaks_t detectPeaks(const Histogram_t &hist)
  {
    const Peaks_t peaks = findPeaks(hist);
    const Peaks_t filtered = filterPeaks(peaks, hist);

    return filtered;
  }

  static Peaks_t findPeaks(const Histogram_t &hist)
  {
    Peaks_t peaks;
    peaks.reserve(hist.size() / 2);

    const size_t n = hist.size() - 1;
    bool ascending = false;

    for(Height_t i = 0; i < n; i++)
    {
      const NumPoints_t curr = hist[i];
      const NumPoints_t succ = hist[i + 1];

      if(curr < succ)
      {
        ascending = true;
        continue;
      }
      if(curr > succ)
      {
        if(ascending)
          peaks.push_back(i);

        ascending = false;
      }
    }
    return peaks;
  }

  static Peaks_t filterPeaks(const Peaks_t &peaks, const Histogram_t &hist)
  {
    Peaks_t filtered;
    filtered.reserve(peaks.size());

    for(auto i: hist){
            const NumPoints_t num_pt = hist[i];
            if(num_pt < 2000){
                continue;
            }
            if((num_pt * 2 - hist[i - 1] - hist[i + 1]) * 2 > num_pt){
                    filtered.push_back(i);
            }
    
  }
    return filtered;
  }

}; // class HeightsHistogram


class PlateausExtraction
{
  const Histogram_t &_heightsHistogram;
  const Peaks_t &_histogramPeaks;

    public:
  PlateausExtraction(const Histogram_t &heightsHistogram, const Peaks_t &histogramPeaks)
  : _heightsHistogram(heightsHistogram),
    _histogramPeaks(histogramPeaks)
  {
  }

  Plateaus_t extractPlateaus(PointsHt_t &pointsHt) const
  {
    Plateaus_t plateaus;
    plateaus.reserve(_histogramPeaks.size());

    PointsHt_t remainder; // pointsHt minus points of all plateaus
    remainder.reserve(pointsHt.size());

    for(Height_t peak : _histogramPeaks)
      plateaus.emplace_back(peak, extractPlateauPoints(peak, pointsHt, remainder));

    // ranges::copy(pointsHt, back_inserter(remainder));
    // pointsHt.clear();

    // TODO use remainder to detect vertical faces

    return plateaus;
  }

    private:
  PointsHt_t extractPlateauPoints(Height_t height, PointsHt_t &pointsHt, PointsHt_t &remainder) const
  {
    // plateau: [heightMin, heightMax] = [height - 1, height] or [height, height + 1]

    Height_t heightMin, heightMax;
    const Height_t pred = height - 1;
    const Height_t succ = height + 1;
    if(_heightsHistogram[pred] > _heightsHistogram[succ])
    {
      heightMin = pred;
      heightMax = height;
    }
    else
    {
      heightMin = height;
      heightMax = succ;
    }
    const size_t plateauSize = _heightsHistogram[heightMin] + _heightsHistogram[heightMax];

    PointsHt_t upper;
    upper.reserve(pointsHt.size());

    // remainder: < heightMin
    // upper: >= heightMin (plateau + above)
    //split(pointsHt, heightMin - 1, remainder, upper);

    PointsHt_t plateauPoints;
    plateauPoints.reserve(plateauSize);
    pointsHt.clear();



    return plateauPoints;
  }

};


    


    // ROS Publisher
    


    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {


        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        pcl::PointXYZ thisPoint;
        
        PointsHt_t pointsHt;
        pointsHt.reserve(temp_cloud->points.size());
        for (PointIndex_t pointIdx = 0; pointIdx < temp_cloud->points.size(); pointIdx++)
        {
                pointsHt.push_back(PointHt{
                                  pointIdx,
                                  static_cast<Height_t>((temp_cloud->points[pointIdx].z - .05) * .01)
                                });



        }
        const auto &config = __processingConfiguration;
        const auto zrange = config.measuringRange.z;
        Histogram_t heightsHistogram;
        Peaks_t histogramPeaks;
        HeightsHistogram::calcHistogram(pointsHt, heightsHistogram, histogramPeaks);

        const PlateausExtraction plateausEx(heightsHistogram, histogramPeaks);
        Plateaus_t plateaus = plateausEx.extractPlateaus(pointsHt);











}





    int main (int argc, char** argv)
    {
        // Initialize the ROS Node "roscpp_pcl_example"
        ros::init (argc, argv, "roscpp_pcl_example");
        ros::NodeHandle nh;
        ros::Publisher img_pub;

        // Print "Hello" message with node name to the terminal and ROS log file
        ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

        // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
        ros::Subscriber sub = nh.subscribe(Point_Topic, 1, cloud_cb);
        

        ros::spin();

        // Success
        return 0;
    }