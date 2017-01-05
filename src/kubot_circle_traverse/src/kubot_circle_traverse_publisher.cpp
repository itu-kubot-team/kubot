#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <queue>

#define CIRCLE_CENTER_X  0.0
#define CIRCLE_CENTER_Y  0.0
#define CIRCLE_RADIUS  10.0
#define CIRCLE_SLICE_WIDTH 1.0

visualization_msgs::Marker point_marker_at(float x, float y);

int main(int argc, char **argv){
    ros::init(argc, argv, "kubot_circle_traverse_publisher");

    ros::NodeHandle n;
    ros::Publisher waypoint_publisher = n.advertise<geometry_msgs::Transform>("/waypoint", 1000);
    ros::Publisher markers_publisher = n.advertise<visualization_msgs::MarkerArray>("/markers", 1000);
    
    std::queue<geometry_msgs::Point> points;
    visualization_msgs::MarkerArray marker_array;
    
    
    geometry_msgs::Point circle_center;
    circle_center.x = CIRCLE_CENTER_X;
    circle_center.y = CIRCLE_CENTER_Y;
    
    double circle_radius = CIRCLE_RADIUS;
    float y_step = CIRCLE_SLICE_WIDTH;
    
    float y_top = circle_center.y + circle_radius;
    float y_bottom = circle_center.y - circle_radius;
    
    /* 
     * Starting from top of square according to base reference
     * Decrease y until to reach the bottom of circle 
     * and find corresponding two x values according those y values
     * finally add all x,y points to the queue 
     */
    float y = y_top - y_step;
    int i = 0;
    while(y > y_bottom){
        float x_abs =  std::sqrt(abs(circle_radius*circle_radius-(y-circle_center.y)*(y-circle_center.y)));
        float x_lhs = circle_center.x - x_abs;
        float x_rhs = circle_center.x + x_abs;
        
        geometry_msgs::Point left_point;
        geometry_msgs::Point right_point;
        
        left_point.x = x_lhs;
        left_point.y = y;
        right_point.x = x_rhs;
        right_point.y = y;
        
        // When going down robot should stay at same side (left or right)
        if(i % 2 == 0){
            points.push(left_point);
            points.push(right_point);
        }
        else{
            //Just change the order of adding   
            points.push(right_point);
            points.push(left_point);
        }
        
        
        visualization_msgs::Marker marker_left = point_marker_at(x_lhs, y);
        visualization_msgs::Marker marker_right = point_marker_at(x_rhs, y);
        marker_array.markers.push_back(marker_left);
        marker_array.markers.push_back(marker_right);
        
        std::cout << "y: " << y << " x1: " << x_rhs << " x_2: " << x_lhs << std::endl;

        y -= y_step;
        i++;
    }
    ROS_INFO("%s", "Publisher");
    
    ros::Rate loop_rate(10);
    while (ros::ok()){

        geometry_msgs::Transform msg;
        //ROS_INFO("%s", "Publisher");
        waypoint_publisher.publish(msg);
        markers_publisher.publish(marker_array);   
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    
}


visualization_msgs::Marker point_marker_at(float x, float y){
  static int id = 1;
    
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/world";
  
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::SPHERE;
  

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
    
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.5;
  marker.color.a = 1.0;
  
  marker.id = id++;
  
  return marker;
}