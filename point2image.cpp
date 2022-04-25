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

    // Topics
    static const std::string Point_Topic = "/camera/depth/color/points";
    const float sensorMinimumRange = 0.4;
    // ROS Publisher



    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {

        std::cout << "hi " << std::endl;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        float Minx , Miny , Minz , Maxx , Maxy , Maxz;
        

        float verticalAngle, horizonAngle, range;
        float p, q , r , th , phi;
        float HoriFOV{69.4}, VerFOV{42.5} , Width{640} , Height{480};
        cv::Mat rangeMat = cv::Mat1f::zeros(Width, Height);
        size_t u, v, index, cloudSize;
        pcl::PointXYZ thisPoint;

        cloudSize = temp_cloud->points.size();

        std::cout << cloudSize << std::endl;

        for (size_t i = 0; i < cloudSize; ++i)
    {
        thisPoint.x = temp_cloud->points[i].x;
        thisPoint.y = temp_cloud->points[i].y;
        thisPoint.z = temp_cloud->points[i].z;
        
        r = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z );

        // th = acos(thisPoint.z / r) ;
        // horizonAngle = th * 180 / M_PI;

        

        q = atan2(thisPoint.y , thisPoint.x );
        horizonAngle = q * 180 / M_PI;

        //std::cout << horizonAngle << "===========++++++++"<< verticalAngle << std::endl;

        p = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y));
        verticalAngle = p * 180 / M_PI;

        //std::cout << p << std::endl;

        if (verticalAngle < -(VerFOV/2) || verticalAngle >= (VerFOV/2) )
            continue;

        v = Height / 2 + verticalAngle * (Height / VerFOV) ;

        if (v < 0 || v >= Height )
            continue;

        // q = atan2(thisPoint.x, thisPoint.y);
        // horizonAngle = q * 180 / M_PI;

        //std::cout << horizonAngle << std::endl;

        if (horizonAngle < -(HoriFOV/2) || horizonAngle >= (HoriFOV/2) )
            continue;

        u = Width / 2 + horizonAngle * (Width / HoriFOV);
        if (u < 0 || u >= Width )
            continue;

        

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);

        std::cout << range << std::endl;
        if (range < sensorMinimumRange)
            continue;

        rangeMat.at<float>(v, u) = 1.0 / (range * cos(p) * std::max(abs(sin(q)), abs(cos(q))));
        //rangeMat *= 0.5;
        rangeMat.convertTo(rangeMat, CV_8U, 255);
        cv::flip(rangeMat, rangeMat, 0);

	    cv::imshow("CSI Camera",rangeMat);
	    int keycode = cv::waitKey(1) & 0xff ; 
            if (keycode == 27) break ;



    }

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