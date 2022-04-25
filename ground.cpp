#include <iostream>


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>




#define VPoint pcl::PointXYZ
typedef pcl::PointCloud<VPoint> PointCloudT;

// using eigen lib
#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
Eigen::Matrix3f cov;
Eigen::Vector4f pc_mean;
Eigen::Vector4f segmentCoefficient;
float curvature;

pcl::PointCloud<VPoint>::Ptr g_seeds_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_all_pc(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr borderPoints(new pcl::PointCloud<VPoint>());


Eigen::Matrix3f eigen_vectors;
Eigen::Vector3f eigen_values;


/*
    @brief Compare function to sort points. Here use z axis.
    @return z-axis accent
*/
bool point_cmp(VPoint a, VPoint b){
    return a.z<b.z;
}

/*
    @brief Ground Plane fitting ROS Node.
    @param Velodyne Pointcloud topic.
    @param Sensor Model.
    @param Sensor height for filtering error mirror points.
    @param Num of segment, iteration, LPR
    @param Threshold of seeds distance, and ground plane distance
    
    @subscirbe:/velodyne_points
    @publish:/points_no_ground, /points_ground
*/
class GroundPlaneFit{
public:
    GroundPlaneFit();
private:
    ros::NodeHandle node_handle_;
    ros::Subscriber points_node_sub_;
    ros::Publisher ground_points_pub_;
    ros::Publisher groundless_points_pub_;
    ros::Publisher all_points_pub_;
    ros::Publisher border_pub_;

    std::string point_topic_;

    int sensor_model_;
    double sensor_height_;
    int num_seg_;
    int num_iter_;
    int num_lpr_;
    double th_seeds_;
    double th_dist_;
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    Eigen::Vector3f dimensions;

    void velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
    void estimate_plane_(void);
    void extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted);
    void createBorder(typename pcl::PointCloud<VPoint>::Ptr &cloud);

    // Model parameter for ground plane fitting
    // The ground plane model is: ax+by+cz+d=0
    // Here normal:=[a,b,c], d=d
    // th_dist_d_ = threshold_dist - d 
    float d_;
    MatrixXf normal_;
    float th_dist_d_;
};    

/*
    @brief Constructor of GPF Node.
    @return void
*/
GroundPlaneFit::GroundPlaneFit():node_handle_("~"){
    // Init ROS related
    ROS_INFO("Inititalizing Ground Plane Fitter...");
    node_handle_.param<std::string>("point_topic", point_topic_, "/camera_crop");
    ROS_INFO("Input Point Cloud: %s", point_topic_.c_str());



    node_handle_.param("sensor_height", sensor_height_, 0.5);
    ROS_INFO("Sensor Height: %f", sensor_height_);

    node_handle_.param("num_seg", num_seg_, 5);
    ROS_INFO("Num of Segments: %d", num_seg_);

    node_handle_.param("num_iter", num_iter_, 100);
    ROS_INFO("Num of Iteration: %d", num_iter_);

    node_handle_.param("num_lpr", num_lpr_, 1);
    ROS_INFO("Num of LPR: %d", num_lpr_);

    node_handle_.param("th_seeds", th_seeds_, 0.1);
    ROS_INFO("Seeds Threshold: %f", th_seeds_);

    node_handle_.param("th_dist", th_dist_, 0.05);
    ROS_INFO("Distance Threshold: %f", th_dist_);

    // Listen to velodyne topic
    points_node_sub_ = node_handle_.subscribe(point_topic_, 2, &GroundPlaneFit::velodyne_callback_, this);
    
    // Publish Init
    std::string no_ground_topic, ground_topic;
    node_handle_.param<std::string>("no_ground_point_topic", no_ground_topic, "/points_no_ground");
    ROS_INFO("No Ground Output Point Cloud: %s", no_ground_topic.c_str());
    node_handle_.param<std::string>("ground_point_topic", ground_topic, "/points_ground");
    ROS_INFO("Only Ground Output Point Cloud: %s", ground_topic.c_str());
    groundless_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 2);
    ground_points_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(ground_topic, 2);
    all_points_pub_ =  node_handle_.advertise<sensor_msgs::PointCloud2>("/all_points", 2);
    border_pub_ =  node_handle_.advertise<sensor_msgs::PointCloud2>("/border", 2);
}

/*
    @brief The function to estimate plane model. The
    model parameter `normal_` and `d_`, and `th_dist_d_`
    is set here.
    The main step is performed SVD(UAV) on covariance matrix.
    Taking the sigular vector in U matrix according to the smallest
    sigular value in A, as the `normal_`. `d_` is then calculated 
    according to mean ground points.
    @param g_ground_pc:global ground pointcloud ptr.
    
*/

void GroundPlaneFit::createBorder(typename pcl::PointCloud<VPoint>::Ptr &cloud)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    transform(0,0)=eigen_vectors(0,2);
    transform(0,1)=eigen_vectors(1,2);
    transform(0,2)=eigen_vectors(2,2);

    transform(1,0)=eigen_vectors(0,1);
    transform(1,1)=eigen_vectors(1,1);
    transform(1,2)=eigen_vectors(2,1);

    transform(2,0)=eigen_vectors(0,0);
    transform(2,1)=eigen_vectors(1,0);
    transform(2,2)=eigen_vectors(2,0);

    transform.col(3) = -(transform*pc_mean);

    transform(3,3)=1;

    pcl::PointCloud<pcl::PointXYZ> tempCloud;

    pcl::transformPointCloud (*cloud,tempCloud, transform);

    std::map<int,Eigen::Vector2f> d1Map;
    std::map<int,Eigen::Vector2f> d2Map;
    std::map<int,Eigen::Vector2i> d1Pos;
    std::map<int,Eigen::Vector2i> d2Pos;

	std::map<int,Eigen::Vector2f>::iterator mapIter;
	std::map<int,Eigen::Vector2i>::iterator mapPosIter;

    for(size_t pointIdx = 0; pointIdx < tempCloud.size(); pointIdx++)
    {
    	float xPoint = tempCloud.at(pointIdx).x;
    	float yPoint = tempCloud.at(pointIdx).y;

    	int d1 = round(xPoint/0.05);
    	int d2 = round(yPoint/0.05);

    	bool foundD1 = false;
    	bool foundD2 = false;

        mapIter = d1Map.find(d2);
        mapPosIter = d1Pos.find(d2);

        if(mapIter == d1Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << xPoint, xPoint;
        	Eigen::Vector2i tempPos;
        	tempPos << pointIdx, pointIdx;
        	d1Map.insert(std::make_pair(d2,tempVec));
        	d1Pos.insert(std::make_pair(d2,tempPos));
        }
        else
        {
        	if(xPoint < mapIter->second[0])
        	{
        		mapIter->second[0] = xPoint;
        		mapPosIter->second[0] = pointIdx;
        	}
        	if(xPoint > mapIter->second[1])
        	{
        		mapIter->second[1] = xPoint;
        		mapPosIter->second[1] = pointIdx;
        	}
        }

        mapIter = d2Map.find(d1);
        mapPosIter = d2Pos.find(d1);

        if(mapIter == d2Map.end() )
        {
        	Eigen::Vector2f tempVec;
        	tempVec << yPoint, yPoint;
        	Eigen::Vector2i tempPos;
        	tempPos << pointIdx, pointIdx;
        	d2Map.insert(std::make_pair(d1,tempVec));
        	d2Pos.insert(std::make_pair(d1,tempPos));
        }
        else
        {
        	if(yPoint < mapIter->second[0])
        	{
        		mapIter->second[0] = yPoint;
        		mapPosIter->second[0] = pointIdx;
        	}
        	if(yPoint > mapIter->second[1])
        	{
        		mapIter->second[1] = yPoint;
        		mapPosIter->second[1] = pointIdx;
        	}
        }
	}
    for(mapPosIter = d1Pos.begin(); mapPosIter != d1Pos.end(); mapPosIter++)
    {
    	borderPoints->points.push_back(cloud->points[mapPosIter->second[0]]);
    	borderPoints->points.push_back(cloud->points[mapPosIter->second[0]]);
    }
    for(mapPosIter = d2Pos.begin(); mapPosIter != d2Pos.end(); mapPosIter++)
    {
    	borderPoints->points.push_back(cloud->points[mapPosIter->second[0]]);
    	borderPoints->points.push_back(cloud->points[mapPosIter->second[0]]);
    }
        borderPoints->header.frame_id = "camera_link";
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*borderPoints , output);
        border_pub_.publish(output);
        g_all_pc->clear();



}




void GroundPlaneFit::estimate_plane_(void){
    // Create covarian matrix in single pass.
    // TODO: compare the efficiency.




    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);

    pcl::eigen33(cov, eigen_vectors, eigen_values);
    // Singular Value Decomposition: SVD

    segmentCoefficient[0] = eigen_vectors(0,0);
    segmentCoefficient[1] = eigen_vectors(1,0);
    segmentCoefficient[2] = eigen_vectors(2,0);
    segmentCoefficient[3] = 0;
    segmentCoefficient[3] = -1 * segmentCoefficient.dot (pc_mean);


    Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
    vp -= pc_mean ;
    float cos_theta = vp.dot (segmentCoefficient);
    if (cos_theta < 0)
    {
    	segmentCoefficient *= -1;
    	segmentCoefficient[3] = 0;
    	segmentCoefficient[3] = -1 * segmentCoefficient.dot (pc_mean);
    }

    // Compute the curvature surface change
    float eig_sum = cov.coeff (0) + cov.coeff (4) + cov.coeff (8);
    if (eig_sum != 0)
		curvature = fabsf (eigen_values[0] / eig_sum);
    else
		curvature = 0;

    std::cout << curvature << std::endl;


    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    // set distance threhold to `th_dist - d`
    th_dist_d_ = th_dist_ - d_;
 
    // return the equation parameters
}


/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::
*/
void GroundPlaneFit::extract_initial_seeds_(const pcl::PointCloud<VPoint>& p_sorted){
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for(int i=0;i<p_sorted.points.size() && cnt<num_lpr_;i++){
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0?sum/cnt:0;// in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for(int i=0;i<p_sorted.points.size();i++){
        if(p_sorted.points[i].z < lpr_height + th_seeds_){
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}

/*
    @brief Velodyne pointcloud callback function. The main GPF pipeline is here.
    PointCloud SensorMsg -> Pointcloud -> z-value sorted Pointcloud
    ->error points removal -> extract ground seeds -> ground plane fit mainloop
*/
void GroundPlaneFit::velodyne_callback_(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    // 1.Msg to pointcloud
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn);
    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud_msg, laserCloudIn_org);
    // For mark ground points and hold all points
    pcl::PointXYZ point;
    for(size_t i=0;i<laserCloudIn.points.size();i++){
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        g_all_pc->points.push_back(point);
    }
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn,indices);
    // 2.Sort on Z-axis value.
    sort(laserCloudIn.points.begin(),laserCloudIn.end(),point_cmp);
    // 3.Error point removal
    // As there are some error mirror reflection under the ground, 
    // here regardless point under 2* sensor_height
    // Sort point according to height, here uses z-axis in default
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for(int i=0;i<laserCloudIn.points.size();i++){
        if(laserCloudIn.points[i].z < -1.5*sensor_height_ && laserCloudIn.points[i].z > 0.2 ){
            it++;
        }else{
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(),it);
    // 4. Extract init ground seeds.
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
    
    // 5. Ground plane fitter mainloop
    for(int i=0;i<num_iter_;i++){
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        //pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(),3);
        int j =0;
        for(auto p:laserCloudIn_org.points){
            points.row(j++)<<p.x,p.y,p.z;
        }
        // ground plane model
        VectorXf result = points*normal_;
        // threshold filter
        for(int r=0;r<result.rows();r++){
            if(result[r]<th_dist_d_){
                //g_all_pc->points[r].label = 1u;// means ground
                g_ground_pc->points.push_back(laserCloudIn_org[r]);

                

            }else{
                //g_all_pc->points[r].label = 0u;// means not ground and non clusterred
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    createBorder(g_ground_pc);
    // publish ground points
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud_msg->header.stamp;
    ground_msg.header.frame_id = in_cloud_msg->header.frame_id;
    ground_points_pub_.publish(ground_msg);
    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*g_not_ground_pc, groundless_msg);
    groundless_msg.header.stamp = in_cloud_msg->header.stamp;
    groundless_msg.header.frame_id = in_cloud_msg->header.frame_id;
    groundless_points_pub_.publish(groundless_msg);
    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud_msg->header.stamp;
    all_points_msg.header.frame_id = in_cloud_msg->header.frame_id;
    all_points_pub_.publish(all_points_msg);
    g_all_pc->clear();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "GroundPlaneFit");
    GroundPlaneFit node;
    ros::spin();

    return 0;

}