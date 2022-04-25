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
    #include <pcl/features/organized_edge_detection.h>
    #include <pcl/features/integral_image_normal.h>
    #include <pcl/features/normal_3d.h>

    // Topics
    static const std::string Point_Topic = "/camera/depth/color/points";
    const float sensorMinimumRange = 0.4;
    ros::Publisher pub1;
    // ROS Publisher

    using namespace pcl;
    


    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {

        std::cout << "hi " << std::endl;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
        PointCloud<Normal>::Ptr normal (new PointCloud<Normal>);
        IntegralImageNormalEstimation<PointXYZRGB, Normal> ne;
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setNormalSmoothingSize (10.0f);
        ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
        ne.setInputCloud (temp_cloud);
        ne.compute (*normal);



        OrganizedEdgeFromNormals<PointXYZRGB, Normal, Label> oed;
  //OrganizedEdgeFromRGBNormals<PointXYZ, Normal, Label> oed;
        oed.setInputNormals (normal);
        oed.setInputCloud (temp_cloud);
        oed.setDepthDisconThreshold (0.5);
        oed.setMaxSearchNeighbors (50);
        oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
        PointCloud<Label> labels;
        std::vector<PointIndices> label_indices;
        oed.compute (labels, label_indices);


        PointCloud<PointXYZRGB>::Ptr occluding_edges (new PointCloud<PointXYZRGB>),
        occluded_edges (new PointCloud<PointXYZRGB>),
        nan_boundary_edges (new PointCloud<PointXYZRGB>),
        high_curvature_edges (new PointCloud<PointXYZRGB>),
        rgb_edges (new PointCloud<PointXYZRGB>);
        copyPointCloud (*temp_cloud, label_indices[0].indices, *nan_boundary_edges);
        copyPointCloud (*temp_cloud, label_indices[1].indices, *occluding_edges);
        copyPointCloud (*temp_cloud, label_indices[2].indices, *occluded_edges);
        copyPointCloud (*temp_cloud, label_indices[3].indices, *high_curvature_edges);
        copyPointCloud (*temp_cloud, label_indices[4].indices, *rgb_edges);


        occluding_edges->header.frame_id = "camera_link";
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*occluding_edges , output);
        pub1.publish(output);






        

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