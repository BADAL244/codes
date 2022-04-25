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
    #include <pcl/filters/extract_indices.h>

    // Topics
    static const std::string Point_Topic = "/camera_crop";
    const float sensorMinimumRange = 0.4;
    ros::Publisher pub1;
    // ROS Publisher
    typedef pcl::PointXYZ PointT;
	typedef std::vector<PointT> VectorT;
    float angle_threshold_ = 1.0;


	std::vector<VectorT> drivable_region(2048);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {

        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
	    
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());


        for (size_t i = 1; i < temp_cloud->size(); ++i){
            auto angle = (temp_cloud->points[i-1].z - temp_cloud->points[i].z) / sqrt(powf(temp_cloud->points[i].x - temp_cloud->points[i-1].x, 2) + powf(temp_cloud->points[i].y - temp_cloud->points[i - 1].y, 2)) * 180.0 / M_PI;
            if(angle < angle_threshold_){
                inliers->indices.push_back(i);
            } 
		    else break;
        }

        extract.setInputCloud(temp_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*filtered_cloud);

        filtered_cloud->header.frame_id = "camera_link";
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud , output);
        pub1.publish(output);

    

        



        // for(int a=0;a<2048;a++) {

        //     for (size_t i = 1; i < temp_cloud->size(); ++i)
        //             {

        //     auto angle = (temp_cloud->points[i-1].z - temp_cloud->points[i].z) / sqrt(powf(temp_cloud->points[i].x - temp_cloud->points[i-1].x, 2) + powf(temp_cloud->points[i].y - temp_cloud->points[i - 1].y, 2)) * 180.0 / M_PI;
        //     std::cout << angle << std::endl;
        // //     if(angle < angle_threshold_) (drivable_region)[a].push_back(temp_cloud->points[i]);
		// // 	else break;
        //           }
		
	    // //     filtered_cloud->points.insert(filtered_cloud->points.end(), (drivable_region)[a].begin(), (drivable_region)[a].end());
        // }
        // filtered_cloud->header.frame_id = "camera_link";
        // sensor_msgs::PointCloud2 output;
        // pcl::toROSMsg(*filtered_cloud , output);
        // pub1.publish(output);


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
        pub1 = nh.advertise<sensor_msgs::PointCloud2> ("voxel_point", 1);


        ros::spin();

        // Success
        return 0;
    }