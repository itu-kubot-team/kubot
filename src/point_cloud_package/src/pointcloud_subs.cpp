
#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>  
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <boost/foreach.hpp>

/////
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/segmentation/sac_segmentation.h>



using std::cout;
using std::endl;

std_msgs::String str;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud,pcl_pc2); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud); 
 
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
   segmentation.setInputCloud(temp_cloud);
   segmentation.setModelType(pcl::SACMODEL_SPHERE);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(1000); 
   segmentation.setOptimizeCoefficients(true);
   segmentation.setRadiusLimits(0.1, 1);
   segmentation.setMaxIterations(10000);
   
   pcl::PointIndices inlierIndices;
   segmentation.segment(inlierIndices, *coefficients);
   
   if (inlierIndices.indices.size() != 0)    
   {
    ROS_INFO("RANSAC found shape with [%d] points", (int)inlierIndices.indices.size());
    cout << "SPHERE!" << endl;
    str.data = "found";
    if((int)temp_cloud->points[inlierIndices.indices[0]].r>(int)temp_cloud->points[inlierIndices.indices[0]].g && (int)temp_cloud->points[inlierIndices.indices[0]].r>(int)temp_cloud->points[inlierIndices.indices[0]].g){
      cout << "RED SPHERE!" << endl;
      str.data = "found red";
      }
    }
    else{
      str.data = "nothing"; 
    }
 
}

void callback2(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud,pcl_pc2); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud); 
 
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
   pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
   segmentation.setInputCloud(temp_cloud);
   segmentation.setModelType(pcl::SACMODEL_CYLINDER);
   segmentation.setMethodType(pcl::SAC_RANSAC);
   segmentation.setDistanceThreshold(1000); 
   segmentation.setOptimizeCoefficients(true);
  // segmentation.setRadiusLimits(0.1, 1);
   segmentation.setMaxIterations(10000);
   
   pcl::PointIndices inlierIndices;
   segmentation.segment(inlierIndices, *coefficients);
   
   if (inlierIndices.indices.size() != 0)    
   {
    ROS_INFO("RANSAC found shape with [%d] points", (int)inlierIndices.indices.size());
    cout << "CYLINDER!" << endl;
    /*for (int c=0; c<coefficients->values.size(); ++c)
        ROS_INFO("Coeff %d = [%f]", (int)c+1, (float)coefficients->values[c]);
    */
    int color_count=0;
    BOOST_FOREACH (const pcl::PointXYZRGB& point, temp_cloud->points)
    {
	if (((unsigned int)point.b > (unsigned int)point.r) && ((unsigned int)point.b > (unsigned int)point.g)){
	  color_count++;
	}
    }
    if (color_count*100/inlierIndices.indices.size()> 50){
      cout << "BLUE CYLINDER!"<<endl;
    }
   }
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "subscribed");
  ros::NodeHandle nh("/");
  ros::Subscriber sub;  
  str.data = "nothing";

 
  //ros::NodeHandle nh2("/");
  //ros::Subscriber sub2;  
  sub =nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback);
  //sub2 = nh2.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback2);
  
  ros::init(argc, argv, "publish");
  ros::Publisher pub;
  pub = nh.advertise<std_msgs::String>("detectionresult", 1000);
  
  
  pub.publish(str);
  ros::Rate spin_rate(1);
  while(ros::ok())
    ros::spin();
  
  return 0;
    
}

