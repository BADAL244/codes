/*
This is an example of how to use the PCL filtering functions in a real robot situation.
This node will take an input depth cloud, and
- run it through a voxel filter to cut the number of points down (in my case, from ~130,000 down to 20,000)
- with a threshold to remove noise (requires minimum 5 input points per voxel)
- then transform it into the robot footprint frame, i.e. aligned with the floor
- subtract the robot footprint (our depth sensor's FOV includes the robot footprint, which changes dynamically with the load)
- subtract points in the ground plane, with different tolerances for near or far
- ground plane by default z = 0, but
- can be calculated at node startup or at any time.
- give you stats on node exit of how long each filter takes, so you can tweak to match your CPU capacity
A few more things could be parameterised, but my intention here more so you can see how it is done without days spent
going through the ROS and PCL documentation.
BSD-licensed, copyright 2009 Tecevo & Josh Marshall
*/

#include <iostream>
#include <signal.h>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h> // robot footprint
#include <geometry_msgs/Polygon.h>
#include <std_srvs/Empty.h>

#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
//#include <message_filters/subscriber.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

// I have added a custom signal handler, so we can print some statistics at shutdown.
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// poly test from https://web.archive.org/web/20120328024410/http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
// nvert           Number of vertices in the polygon. Whether to repeat the first vertex at the end is discussed below.
// vertx, verty    Arrays containing the x- and y-coordinates of the polygon's vertices.
// testx, testy    X- and y-coordinate of the test point.
inline int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
	 (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

class DepthPreprocess
{
    public:
        DepthPreprocess(tf::TransformListener& tf): 
            tf_listener(tf),
            footprint_xs_({-1., -1., +0.5, +1.,  +1. , +0.5}),
            footprint_ys_({-1., +1., +1.,  +0.5, -0.5, -1.}),
            fp_min_x_(-1.), fp_max_x_(+1.), fp_min_y_(-1.), fp_max_y_(+1.),
            groundplane_model_(Eigen::Vector4f(0., 0., 1., 0.)),
            update_groundplane_(true)
        {
            nh_private_ = ros::NodeHandle("~");
                
            pub_points_ = nh_.advertise<PointCloud>("points_filtered", 1);
            sub_points_ = nh_.subscribe<PointCloud>("points", 1, &DepthPreprocess::points_callback, this);
            
            sub_footprint_ = nh_.subscribe<geometry_msgs::PolygonStamped>("footprint", 1, &DepthPreprocess::footprint_callback, this);
            
            nh_private_.param<bool>("enable_voxel_filter", enable_voxel_filter_, true);
            nh_private_.param<bool>("enable_transform", enable_transform_, true);
            nh_private_.param<bool>("enable_footprint_filter", enable_footprint_filter_, true);
            nh_private_.param<bool>("enable_groundplane_filter", enable_groundplane_filter_, true);
            
            nh_private_.param<std::string>("base_frame", base_frame_, "base_footprint"); // frame of the footprint
            nh_private_.param<float>("max_distance", max_distance_, 3.0); // maximum distance of the points from the sensor, pre-filtered before the voxels 
            
            nh_private_.param<float>("voxel_size_", voxel_size_, 0.05); // downsample voxel leaf size, 5cm is good 
            nh_private_.param<int>("voxel_threshold", thresh_, 5); // minimum number of points that have to occupy a voxel in order for it to survive the downsample
            
            nh_private_.param<float>("groundplane_threshold", groundplane_threshold_, 0.03); // distance from ground plane for points to be culled
            
            //
            update_groundplane_server = nh_.advertiseService("update_groundplane", &DepthPreprocess::update_groundplane_service, this);
            
            if (nh_private_.hasParam("groundplane_model"))
            {
                std::vector<float> groundplane_model_param;
                nh_private_.param<std::vector<float>>("groundplane_model", groundplane_model_param);
                groundplane_model_ = Eigen::Vector4f(groundplane_model_param[0], groundplane_model_param[1], groundplane_model_param[2], groundplane_model_param[3]);
            }
            //else {
                // default ground plane is aligned with base_footprint
            //    groundplane_model_ = Eigen::Vector4f(0., 0., 1., 0.);
            //}
            //nh_private.param<int>("min_points", this->min_points, 1000); // minimum number of points post-filtering to ensure depth camera is okay
            
            //ROS_INFO("Started depth voxel filter; leaf size %02f, voxel thresh_old %d, delay %d msec", this->voxel_size_, this->thresh_, this->delay_usec / 1000);
        }
        
        void footprint_callback(const geometry_msgs::PolygonStamped::ConstPtr& footprint_msg) 
        {
            ROS_INFO("Received new footprint.");
            //std::vector<geometry_msgs::Point32> footprint_points = footprint_msg->polygon.points;
            footprint_xs_.clear();
            footprint_ys_.clear();
            fp_min_x_ = 0.;
            fp_max_x_ = 0.;
            fp_min_y_ = 0.;
            fp_max_y_ = 0.;
            
            for(geometry_msgs::Point32 pt: footprint_msg->polygon.points) {
                // add x & y vertices to the arrays used for testing
                footprint_xs_.push_back(pt.x);
                footprint_ys_.push_back(pt.y);
                // and build bounding box
                fp_min_x_ = std::min(fp_min_x_, pt.x);
                fp_max_x_ = std::max(fp_max_x_, pt.x);
                fp_min_y_ = std::min(fp_min_y_, pt.y);
                fp_max_y_ = std::max(fp_max_y_, pt.y);
            }
        }

        void points_callback(const PointCloud::ConstPtr& input_cloud)
        {
            iter_count++;
            double t_start = ros::Time::now().toSec();
            double t_voxel, t_transform, t_footprint, t_groundplane;
            
            if (!enable_voxel_filter_)
            {
                pub_points_.publish(input_cloud);
                return;
            }
            // Downsample using a VoxelGrid filter to reduce the amount of data we're dealing with.
            PointCloud::Ptr voxel_filtered_cloud (new PointCloud);
            voxel_filter(input_cloud, voxel_filtered_cloud);

            t_voxel = ros::Time::now().toSec();
            time_voxel += t_voxel - t_start;
            
            if (!enable_transform_)
            {
                pub_points_.publish(voxel_filtered_cloud);
                return;
            }

            // Transform to robot frame
            PointCloud::Ptr transformed_cloud (new PointCloud);
            transform_to_local_frame(voxel_filtered_cloud, transformed_cloud);
            
            t_transform = ros::Time::now().toSec();
            time_transform += t_transform - t_voxel;
            
            if (!enable_footprint_filter_)            
            {
                pub_points_.publish(transformed_cloud);
                return;
            }
            
            // Remove points within the footprint
            PointCloud::Ptr defootprinted_cloud (new PointCloud);
            subtract_footprint(transformed_cloud, defootprinted_cloud);
            
            t_footprint = ros::Time::now().toSec();
            time_footprint += t_footprint - t_transform;
            
            if (!enable_groundplane_filter_)            
            {
                pub_points_.publish(defootprinted_cloud);
                return;
            }
            
            // Remove the ground plane
            PointCloud::Ptr degroundplaned_cloud (new PointCloud);
            remove_groundplane(transformed_cloud, degroundplaned_cloud);
            
            t_groundplane = ros::Time::now().toSec();
            time_groundplane += t_groundplane - t_footprint;
            
            pub_points_.publish(degroundplaned_cloud);
            
            // If we need to update the ground plane, do it now after the last publish
            if (update_groundplane_) 
            {
                update_groundplane_model(defootprinted_cloud);
            }
        }
        
        bool voxel_filter(const PointCloud::ConstPtr& input_cloud, PointCloud::Ptr& output_cloud) 
        {
            pcl::VoxelGrid<PointXYZ> vox;
            vox.setInputCloud(input_cloud);
            // The leaf size is the size of voxels pretty much. Note that this value affects what a good thresh_old value would be.
            vox.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
            // I limit the overall volume being considered so lots of "far away" data that is just terrible doesn't even have to be considered.
            vox.setFilterFieldName("z");
            vox.setFilterLimits(0.0, max_distance_);
            // The line below is perhaps the most important as it reduces ghost points.
            vox.setMinimumPointsNumberPerVoxel(thresh_);
            
            vox.filter(*output_cloud);
            //ROS_INFO("cloud time %lu output %lu", input_cloud->header.stamp, output_cloud->header.stamp);
            
            return true;
        }
        
        bool transform_to_local_frame(const PointCloud::ConstPtr& input_cloud, PointCloud::Ptr& output_cloud)
        {
            // transform to 'base_footprint'?
            tf::StampedTransform transform;
            try
            {
                tf_listener.waitForTransform(base_frame_, input_cloud->header.frame_id, pcl_conversions::fromPCL(input_cloud->header.stamp), ros::Duration(0.2));
                tf_listener.lookupTransform(base_frame_, input_cloud->header.frame_id, pcl_conversions::fromPCL(input_cloud->header.stamp), transform); // 
                //tf_listener.lookupTransform(base_frame, input_cloud->header.frame_id, ros::Time(0), transform);
            }
            catch (tf::TransformException ex)
            {
                ROS_ERROR("Failed to lookup transform to base frame from frame %s", input_cloud->header.frame_id.c_str());
                ROS_ERROR("%s",ex.what());
                return false;
            }
            
            // Transform to the dock frame (link or footprint?)
            Eigen::Matrix4f matrix_transform;
            pcl_ros::transformAsMatrix(transform, matrix_transform);
            
            pcl::transformPointCloud (*input_cloud, *output_cloud, matrix_transform);
            output_cloud->header.frame_id = base_frame_;   
            //output_cloud->header = input_cloud->header;
            return true;
        }
        
        bool subtract_footprint(const PointCloud::ConstPtr& input_cloud, PointCloud::Ptr& output_cloud) 
        {
            if (footprint_xs_.size() == 0) {
                ROS_WARN_THROTTLE(5, "No footprint received, passing full point cloud through.");
                //return false;
            }
            
            //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            //int pcount = input_cloud->size();
            
            //int cloud_size = input_cloud->size();
            int poly_size = footprint_xs_.size();
            float* poly_xs = footprint_xs_.data();
            float* poly_ys = footprint_ys_.data();
            //for(int idx=0; idx < cloud_size; idx++) {
            for (PointXYZ pt: input_cloud->points) {
                // bounding box test first, since most points will be outside bb of footprint
                // then
                // poly test from https://web.archive.org/web/20120328024410/http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
                // nvert           Number of vertices in the polygon. Whether to repeat the first vertex at the end is discussed below.
                // vertx, verty    Arrays containing the x- and y-coordinates of the polygon's vertices.
                // testx, testy    X- and y-coordinate of the test point.
                // inline int pnpoly(int nvert, float *vertx, float *verty, float testx, float testy)
                if (pt.x < fp_min_x_ || pt.x > fp_max_x_ || pt.y < fp_min_y_ || pt.y > fp_max_y_ || // bounding box test
                    !pnpoly(poly_size, poly_xs, poly_ys, pt.x, pt.y))                               // poly test
                {
                    output_cloud->push_back(PointXYZ(pt.x, pt.y, pt.z));
                }
            }
            
            output_cloud->header = input_cloud->header;
            output_cloud->height = 1;
            output_cloud->width = output_cloud->points.size();
            
            return true;
        }
        
        bool remove_groundplane(const PointCloud::ConstPtr& input_cloud, PointCloud::Ptr& output_cloud) 
        {
            //pcl::copyPointCloud(*input_cloud, *output_cloud);
            pcl::SampleConsensusModelPlane<pcl::PointXYZ> plane_model = pcl::SampleConsensusModelPlane<pcl::PointXYZ>(input_cloud);
            
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            //   selectWithinDistance (const Eigen::VectorXf &model_coefficients, 
            //                const double threshold, 
            //                std::vector<int> &inliers) override;
            //plane_model.selectWithinDistance(groundplane_model_, groundplane_threshold_, inliers->indices);
            selectOutsideGroundPlane(input_cloud, groundplane_model_, groundplane_threshold_, inliers->indices);
            
            //pcl::copyPointCloud<pcl::PointXYZ>(*input_cloud, inliers, *output_cloud);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(input_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true); // points not matching the ground plane
            
            extract.filter (*output_cloud);
            
            return true;
        }
        
        // a rewritten pcl::SampleConsensusModelPlane::selectWithinDistance that varies the threshold as we move away from the robot
        // in order to compensate for greater noise
        // https://github.com/PointCloudLibrary/pcl/blob/master/sample_consensus/include/pcl/sample_consensus/impl/sac_model_plane.hpp
        void selectOutsideGroundPlane(const PointCloud::ConstPtr& input_cloud, Eigen::Vector4f& plane_coefficients, float threshold, std::vector<int> &inliers)
        {
            int nr_p = 0;
            inliers.resize(input_cloud->size());
            
            float threshold_close    = threshold;
            float threshold_far      = 0.1;
            float threshold_boundary = 1.5 * 1.5; // pre-squared
            
            // Iterate through the 3d points and calculate the distances from them to the plane
            for (size_t i = 0; i < input_cloud->size(); ++i)
            {
              // Calculate the distance from the point to the plane normal as the dot product
              // D = (P-A).N/|N|
              Eigen::Vector4f pt (input_cloud->points[i].x,
                                  input_cloud->points[i].y,
                                  input_cloud->points[i].z,
                                  1);
    
              float distance = fabsf(plane_coefficients.dot(pt));
              
              // check to see whether the point is near or far from us
              float source_distance = std::pow(input_cloud->points[i].x, 2) + std::pow(input_cloud->points[i].y, 2);
              bool near = source_distance < threshold_boundary;
                  
              if ((near && distance < threshold_close)  || (!near && distance < threshold_far)) // (near ? threshold_close : threshold_far))
              {
                // Returns the indices of the points whose distances are smaller than the threshold
                inliers[nr_p] = i;
                ++nr_p;
              }
            }
            inliers.resize(nr_p);
        }
        
        
        bool update_groundplane_model(const PointCloud::ConstPtr& input_cloud) {
            // the input cloud should already have been transformed to the ground plane
            
            // assume reasonably close to the ground plane already, so remove points < 0.1m vertically.
            // and crop out points outside a 1.5m xy box, so we get better accuracy
            pcl::CropBox<pcl::PointXYZ> crop;
            crop.setInputCloud (input_cloud);
            crop.setMin(Eigen::Vector4f(-1.5, -1.5, -0.1, 0.));
            crop.setMax(Eigen::Vector4f(+1.5, +1.5, +0.1, 0.));
            PointCloud::Ptr potential_ground_points (new PointCloud);
            crop.filter (*potential_ground_points);
            
            // check for enough points.
            if (potential_ground_points.size() < 500)
            {
                ROS_WARN("Less than 500 points provided to update_groundplane_model, not updating until next pointcloud.");
                return false;
            }
            
            // run a RANSAC over remaining points to find the best plane.
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients (true); // Optional
            // Mandatory
            seg.setModelType (pcl::SACMODEL_PLANE);
            seg.setMethodType (pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.025); // allow for 25mm of noise
            seg.setInputCloud (potential_ground_points);
            seg.segment (*inliers, *coefficients);
            
            if (inliers->indices.size () == 0)
            {
                // failure
                //res.success = false;
                //res.message = "Could not estimate a planar model for the given dataset.";
                ROS_WARN("Could not estimate a planar model for the given dataset, keeping existing model.");
                return false;
            }
            
            ROS_INFO("%s: New ground plane model: %.3f %.3f, %.3f, %.3f.", ros::this_node::getName().c_str(),
                coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
            ROS_INFO("%lu inliers of %lu points within z [-0.1, 0.1]: %.1f %%", 
                inliers->indices.size(), potential_ground_points->size(), 100.0 * inliers->indices.size() / potential_ground_points->size());
            
            // Check to make sure our ground plane model is credible
            bool validity_checks = true;
            if (inliers->indices.size() < 300) 
            {
                ROS_WARN("Less than 300 points in ground plane model, not updating until next pointcloud.");
                validity_checks = false;
            }
            if (std::abs(coefficients->values[0]) > 0.1 ||  // x
                std::abs(coefficients->values[1]) > 0.1 ||  // y
                std::abs(coefficients->values[2]-1.0) > 0.1) // z
            { 
                ROS_WARN("Ground plane is significantly off horizontal, not updating until next pointcloud.");
                validity_checks = false;
            }
            if (std::abs(coefficients->values[3]) > 0.05) 
            {
                ROS_WARN("Ground plane has an altitude that exceeds 50mm from the base, not updating until next pointcloud.");
                validity_checks = false;
            }
            
            if (validity_checks) 
            {
                ROS_WARN("Ground plane updated.");
                groundplane_model_ = Eigen::Vector4f(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
                update_groundplane_ = false; // only do once per set
            }
            return validity_checks;
        }
        
        bool update_groundplane_service(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
        {
           update_groundplane_ = true;
           return true;
        }
        
        void print_runtimes() 
        {
            double time_total = time_voxel + time_transform + time_footprint + time_groundplane;
            ROS_INFO("Preprocess runtime per time (ms): %.3f", 1000.*time_total/iter_count);
            ROS_INFO("time per iter (ms): Voxel: %.3f Transform: %.3f Footprint subtract: %.3f Ground plane: %.3f", 
                    1000.*time_voxel/iter_count, 1000.*time_transform/iter_count, 1000.*time_footprint/iter_count, 1000.*time_groundplane/iter_count);
            ROS_INFO("percent: Voxel: %.1f%% Transform: %.1f%% Footprint subtract: %.1f%% Ground plane: %.1f%%",
                    time_voxel/time_total*100, time_transform/time_total*100, time_footprint/time_total*100, time_groundplane/time_total*100);
            
        }
        
    private:
        //tf::TransformListener tf;
        
        ros::NodeHandle nh_;
        ros::NodeHandle nh_private_;
        ros::Publisher pub_points_;
        ros::Subscriber sub_points_;
        ros::Subscriber sub_footprint_;
        
        ros::ServiceServer update_groundplane_server;
        
        //tf2_ros::Buffer tf_buffer;
        tf::TransformListener& tf_listener;

        bool enable_voxel_filter_;
        bool enable_transform_;
        bool enable_footprint_filter_;
        bool enable_groundplane_filter_;
        
        bool update_groundplane_;
         
        int thresh_;
        //int min_points;
        float max_distance_;
        float voxel_size_;
        
        std::string base_frame_;
        
        //std::vector<geometry_msgs::Point32> footprint_points;
        std::vector<float> footprint_xs_;
        std::vector<float> footprint_ys_;
        float fp_min_x_, fp_max_x_, fp_min_y_, fp_max_y_; // bounding box of footprint
        
        Eigen::Vector4f groundplane_model_;
        float groundplane_threshold_;
        
        // runtime counting for profiling
        double time_voxel, time_transform, time_footprint, time_groundplane;
        int iter_count;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "voxel_filtering", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);
    
    tf::TransformListener tf(ros::Duration(10));
    DepthPreprocess dp = DepthPreprocess(tf);
    
    while(!g_request_shutdown && ros::ok()) 
    {
        ros::spinOnce();
        usleep(1000);
    }
    
    dp.print_runtimes();
    
    ros::shutdown();
    
    return 0;
}