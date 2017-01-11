
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
ros::Publisher pub;
int found;

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
   segmentation.setRadiusLimits(0.4, 1);
   segmentation.setMaxIterations(1000);
   
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
      found = 1;
      }
    }
 
    std_msgs::String nstr;
    nstr.data = str.data;
    pub.publish(nstr);
	
}

void callback2(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud,pcl_pc2); 
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud); 

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::SACSegmentation<pcl::PointXYZRGB> segmentation; 
      segmentation.setOptimizeCoefficients(true); 
      segmentation.setModelType(pcl::SACMODEL_PLANE); 
      segmentation.setMethodType(pcl::SAC_RANSAC); 
      segmentation.setDistanceThreshold(0.01);
      segmentation.setMaxIterations(200); 
      segmentation.setInputCloud(temp_cloud);
      segmentation.setEpsAngle (0.1);

   pcl::PointIndices inlierIndices;
   segmentation.segment(inlierIndices, *coefficients);
   
    
    
   if (inlierIndices.indices.size() != 0)    
   {
    int size = (int)inlierIndices.indices.size();
    ROS_INFO("RANSAC found shape with [%d] points", size);
    float y1 = temp_cloud->points[inlierIndices.indices[0]].y;
    float z1 = temp_cloud->points[inlierIndices.indices[0]].z;
    float y2 =temp_cloud->points[inlierIndices.indices[size/2]].y;
    float z2 =temp_cloud->points[inlierIndices.indices[size/2]].z;
    
    
    float transformedy1 = y1*cos(-1)-z1*sin(-1);
    float transformedy2 = y2*cos(-1)-z2*sin(-1);
    
    cout << temp_cloud->points[inlierIndices.indices[0]].x << "  " << y1 << "  " << temp_cloud->points[inlierIndices.indices[0]].z << endl;
    cout << temp_cloud->points[inlierIndices.indices[size/2]].x << "  " << y2 << "  " << temp_cloud->points[inlierIndices.indices[size/2]].z << endl;
    
    cout << transformedy1 << endl;
    cout << transformedy2 << endl;
     cout << (int)temp_cloud->points[inlierIndices.indices[size/2]].r << endl;
     cout << (int)temp_cloud->points[inlierIndices.indices[size/2]].g << endl;
     cout << (int)temp_cloud->points[inlierIndices.indices[size/2]].b << endl;
    if(fabs(transformedy2- transformedy1) < 0.1 && transformedy1 < 1.3)
    {cout << "CUBE!" << endl;
     str.data = "found cube";
    cout << "blabla" << endl;
     if((int)temp_cloud->points[inlierIndices.indices[size/2]].g>(int)temp_cloud->points[inlierIndices.indices[size/2]].b && (int)temp_cloud->points[inlierIndices.indices[size/2]].g>(int)temp_cloud->points[inlierIndices.indices[size/2]].r){
      cout << "GREEN CUBE!" << endl;
      str.data = "found green";
      }     
    }  
   }
}


void callback0(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  callback(cloud);
  callback2(cloud);
  
   
   std_msgs::String nstr;
   nstr.data = str.data;
   pub.publish(nstr);
        
  
}

int main(int argc, char **argv) {
  
  ros::init(argc, argv, "subscribed");
  ros::NodeHandle nh("/");
  ros::Subscriber sub;  
  str.data = "nothing";
  found = 0;
 
  //ros::NodeHandle nh2("/");
  //ros::Subscriber sub2;  
  sub =nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback0);
  //sub2 = nh2.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, callback2);
  
  ros::init(argc, argv, "publish");
  pub = nh.advertise<std_msgs::String>("detectionresult", 1000);
  
  
  ros::Rate spin_rate(1);
  while(ros::ok())
    ros::spin();
  
  return 0;
    
}

