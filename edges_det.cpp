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
    #include <iostream>
    #include <Eigen/Eigenvalues>
    #include <Eigen/Dense>
    #include <vector>
    #include <math.h>

    #include <fstream>
    #include <string>
    #include <vector>
    #include <pcl/io/io.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/point_types.h>
    #include <pcl/features/integral_image_normal.h>
    #include <pcl/features/normal_3d.h>
    #include <pcl/common/common_headers.h>
    #include <pcl/features/integral_image_normal.h>
    #include <pcl/features/normal_3d.h>
    #include <pcl/visualization/cloud_viewer.h>
    #include <pcl/filters/passthrough.h>
    #include <pcl/ModelCoefficients.h>
    #include <pcl/filters/project_inliers.h>
    #include <pcl/features/shot_omp.h>
    #include "pcl/features/fpfh.h"
    #include <pcl/io/ply_io.h>
    #include <pcl/filters/extract_indices.h>

    // Topics
    static const std::string Point_Topic = "/camera_crop";
    //static const std::string Point_Topic = "/cloud_pcd";

    
    // ROS Publisher
    using namespace std;
    using namespace Eigen;

    ros::Publisher pub1;

    // double dist(pcl::PointXYZRGB a, pcl::PointXYZRGB b);
    // std::vector<double> numPoints(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 , typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
    // void crop_cloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud);

    // std::vector<double> ret;
    // void print_cloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        std::cout << "hi " << std::endl;
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

        //std::cout << "Number of points in the Cube Input cloud is:"<< cloud->points.size() << std::endl;
    
	    int KNumbersNeighbor = 200; // numbers of neighbors 7 , 120
	    std::vector<int> NeighborsKNSearch(KNumbersNeighbor);
	    std::vector<float> NeighborsKNSquaredDistance(KNumbersNeighbor);

	    int* NumbersNeighbor = new  int [cloud ->points.size ()];

	    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	    kdtree.setInputCloud (cloud);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	    pcl::PointXYZRGB searchPoint;
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	    
	    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  	  

	    double* SmallestEigen = new  double [cloud->points.size() ];
	    double* MiddleEigen = new  double [cloud->points.size() ];
	    double* LargestEigen = new  double [cloud->points.size() ];

	    double* DLS = new  double [cloud->points.size() ];
	    double* DLM = new  double [cloud->points.size() ];
	    double* DMS = new  double [cloud->points.size() ];
	    double* Sigma = new  double [cloud->points.size() ];

        for (size_t i = 0; i < cloud ->points.size (); ++i) {

            searchPoint.x =   cloud->points[i].x;
	        searchPoint.y =   cloud->points[i].y;
	        searchPoint.z =   cloud->points[i].z;

	    if ( kdtree.nearestKSearch (searchPoint, KNumbersNeighbor, NeighborsKNSearch, NeighborsKNSquaredDistance) > 0 ) {
		        NumbersNeighbor[i]= NeighborsKNSearch.size (); }
	    else { NumbersNeighbor[i] = 0; }

	    float Xmean; float Ymean; float Zmean;
	    float sum= 0.00;
        // Computing Covariance Matrix
	    for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
            sum += cloud->points[ NeighborsKNSearch[ii] ].x; }
	        Xmean = sum / NumbersNeighbor[i] ;
	        sum= 0.00;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += cloud->points[NeighborsKNSearch[ii] ].y;}
			Ymean = sum / NumbersNeighbor[i] ;
		    sum= 0.00;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += cloud->points[NeighborsKNSearch[ii] ].z;}
			Zmean = sum / NumbersNeighbor[i] ;

			float	CovXX;  float CovXY; float CovXZ; float CovYX; float CovYY; float CovYZ; float CovZX; float CovZY; float CovZZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].x - Xmean )  );}
			CovXX = sum / ( NumbersNeighbor[i]-1) ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );}
			CovXY = sum / ( NumbersNeighbor[i]-1) ;

			CovYX = CovXY ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].x - Xmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovXZ= sum / ( NumbersNeighbor[i]-1) ;

			CovZX = CovXZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].y - Ymean )  );}
			CovYY = sum / ( NumbersNeighbor[i]-1) ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].y - Ymean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovYZ = sum / ( NumbersNeighbor[i]-1) ;

			CovZY = CovYZ;

			sum = 0.00 ;
			for (size_t ii = 0; ii < NeighborsKNSearch.size (); ++ii){
			sum += ( (cloud->points[NeighborsKNSearch[ii] ].z - Zmean ) * ( cloud->points[NeighborsKNSearch[ii] ].z - Zmean )  );}
			CovZZ = sum / ( NumbersNeighbor[i]-1) ;

   // Computing Eigenvalue and EigenVector
//    Matrix3f Cov;
//    Cov << CovXX, CovXY, CovXZ, CovYX, CovYY, CovYZ, CovZX, CovZY, CovZZ;

   Matrix3f Cov;
   Cov << 0, CovXY, 0.0, CovYX, CovYY, 0.0, 0.0, CovZY, CovZZ;

  SelfAdjointEigenSolver<Matrix3f> eigensolver(Cov);
  if (eigensolver.info() != Success) abort();

  double EigenValue1 = eigensolver.eigenvalues()[0];
  double EigenValue2 = eigensolver.eigenvalues()[1];
  double EigenValue3 = eigensolver.eigenvalues()[2];

  double Smallest = 0.00; double Middle = 0.00; double Largest= 0.00;
  if (EigenValue1<  EigenValue2 ) { Smallest =  EigenValue1 ; } else { Smallest = EigenValue2 ; }
  if (EigenValue3<  Smallest ) { Smallest =  EigenValue3 ; }


  if(EigenValue1 <= EigenValue2 && EigenValue1 <= EigenValue3) {
	  Smallest = EigenValue1;
  if(EigenValue2 <= EigenValue3) {Middle = EigenValue2; Largest = EigenValue3;}
  else {Middle = EigenValue3; Largest = EigenValue2;}
  }

  if(EigenValue1 >= EigenValue2 && EigenValue1 >= EigenValue3)
  {
	  Largest = EigenValue1;
  if(EigenValue2 <= EigenValue3) { Smallest = EigenValue2; Middle = EigenValue3; }
  else {Smallest = EigenValue3; Middle = EigenValue2;}
  }

  if ((EigenValue1 >= EigenValue2 && EigenValue1 <= EigenValue3) || (EigenValue1 <= EigenValue2 && EigenValue1 >= EigenValue3))
  {
	  Middle = EigenValue1;
  if(EigenValue2 >= EigenValue3){Largest = EigenValue2; Smallest = EigenValue3;}
  else{Largest = EigenValue3; Smallest = EigenValue2;}
  }

    SmallestEigen[i]= Smallest ;
    MiddleEigen[i]= Middle;
    LargestEigen[i]= Largest;

    DLS[i] =    std::abs ( SmallestEigen[i] / LargestEigen[i]) ;          // std::abs ( LargestEigen[i] -  SmallestEigen[i] ) ;
    DLM[i] = std::abs ( MiddleEigen[i] /  LargestEigen[i]) ;             // std::abs (  LargestEigen[i] - MiddleEigen[i] ) ;
    DMS[i] = std::abs ( SmallestEigen[i] / MiddleEigen[i]) ;       // std::abs ( MiddleEigen[i] -  SmallestEigen[i] ) ;
    Sigma[i] = (SmallestEigen[i] ) / ( SmallestEigen[i] + MiddleEigen[i] + LargestEigen[i] ) ;
}

	double MaxD=0.00 ;
	double MinD= cloud ->points.size ();
	int Ncolors=256;
	  

	for (size_t i = 0; i < cloud ->points.size (); ++i) {
	  	if (  Sigma [i] < MinD) MinD= Sigma [i];
	  	if (  Sigma[i] > MaxD) MaxD = Sigma [i];
	  }

	std::cout<< " Minimum is :" << MinD<< std::endl;
	std::cout<< " Maximum  is :" << MaxD << std::endl;

    int Edgepoints = 0;
	   

	// for (size_t i = 0; i < cloud ->points.size (); ++i) {
	//     	cloud->points[i].r = 240;
	//     	cloud->points[i].g =  230 ;
	//     	cloud->points[i].b =  140;
	//     }

	int level = 0;
	float step = ( ( MaxD -  MinD) / Ncolors ) ;
    for (size_t i = 0; i < cloud ->points.size (); ++i) {
	  if ( Sigma [i] > ( MinD + ( 6* step) ) ) {  
	    // cloud->points[i].r = 255;
	    // cloud->points[i].g =  0 ;
	    // cloud->points[i].b =  0;
		inliers->indices.push_back(i);
        Edgepoints ++;
        }
     }
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*new_cloud);

    // ret = numPoints(cloud , new_cloud);
    // std::cout << ret[0]  << "2nd one" << ret[1] << std::endl;
    
    // if(ret[1] <= 0.2 * ret[0] )
    // {
    //   crop_cloud(*new_cloud);
    // }
    

    //std::cout<< " Number of Edge points  is :" << Edgepoints << std::endl;       
    new_cloud->header.frame_id = "camera_link";
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*new_cloud , output);
    pub1.publish(output);

}

// double dist(pcl::PointXYZRGB a, pcl::PointXYZRGB b)
// {
//     double dx = a.x-b.x;
//     double dy = a.y-b.y;
//     double dz = a.z-b.z;
//     return std::sqrt(dx*dx+dy*dy+dz*dz);
// }




// std::vector<double> numPoints(typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
// {
//     std::cout << cloud1->points.size() << std::endl;
//     std::vector<double> vec;
//     vec.push_back(cloud1->points.size());
//     vec.push_back(cloud2->points.size());
//     return vec;
// }

// void crop_cloud(pcl::PointCloud<pcl::PointXYZRGB> &pcl_cloud)
// {

//   pcl::PointCloud<pcl::PointXYZRGB>::iterator i;
//   float max_z = .3  ;

//   for (i = pcl_cloud.begin(); i != pcl_cloud.end();)
//   {

//     bool remove_point = 0;

//     if (i->z > max_z)
//     {
//       remove_point = 1;
//     }
//     if(dist(*(i) , *(i-1))> 0.1)
//     {
//       remove_point = 1;
//     }

//     // if (sqrt(pow(i->x,2) + pow(i->y,2)) > max_radius)
//     // {
//     //   remove_point = 1;
//     // }
//     if (remove_point == 1)
//     {
//       i = pcl_cloud.erase(i);
//     }
//     else
//     {
//       i++;
//     }

//   }

// }

// void print_cloud( pcl::PointCloud<pcl::PointXYZRGB> &cloud){

//     pcl::PointCloud<pcl::PointXYZRGB>::iterator it;

//     for (it = cloud.begin(); it != cloud.end();)
//     {
//       std::cout << "print  cloud" << *(it) << std::endl;
//     }


// }


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