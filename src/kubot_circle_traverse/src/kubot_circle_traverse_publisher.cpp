#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <queue>

#define CIRCLE_CENTER_X  0.0            // meters
#define CIRCLE_CENTER_Y  0.0            // meters
#define CIRCLE_RADIUS  8.0            // meters
#define CIRCLE_SLICE_WIDTH 1.0          // meters
#define PROXIMITY_THERESHOLD 0.5        // meters
#define ROS_RATE 10

visualization_msgs::Marker point_marker_at(float x, float y);
float normalize_angle(float angle_in_degree);
bool are_two_points_close(double x_1, double y_1, double x_2, double y_2);
bool are_two_points_close(double x_1, double y_1, double x_2, double y_2, double threshold);
double distance_between_two_points(double x_1, double y_1, double x_2, double y_2);
tf::StampedTransform robot_pose;

int main(int argc, char **argv){
    ros::init(argc, argv, "kubot_circle_traverse_publisher");

    ros::NodeHandle n;
    ros::Publisher waypoint_publisher = n.advertise<geometry_msgs::PointStamped>("/waypoint", 1000);
    ros::Publisher markers_publisher = n.advertise<visualization_msgs::MarkerArray>("/markers", 1000);
    
    std::queue<geometry_msgs::Point> points;
    visualization_msgs::MarkerArray marker_array;
    
    /**** CALCULATION OF POINTS ON CIRCLE ****/
    std::cout << "#### CALCULATION OF POINTS ON CIRCLE ####" << std::endl;
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
        
        // When going down on circles edges robot should stay at same side (left or right)
        if(i % 2 == 0){
            points.push(left_point);
            points.push(right_point);
            std::cout  << 2*(i+1)-1 << ". Point: {x:" << x_lhs << ", y:" << y << "}" << std::endl;
            std::cout  << 2*(i+1) << ". Point: {x:" << x_rhs << ", y:" << y << "}" << std::endl;
        }
        else{
            //Just change the order of adding   
            points.push(right_point);
            points.push(left_point);
            std::cout  << 2*(i+1)-1 << ". Point: {x:" << x_rhs << ", y:" << y << "}" << std::endl;
            std::cout  << 2*(i+1) << ". Point: {x:" << x_lhs << ", y:" << y << "}" << std::endl;
        }
        
        visualization_msgs::Marker marker_left = point_marker_at(x_lhs, y);
        visualization_msgs::Marker marker_right = point_marker_at(x_rhs, y);
        marker_array.markers.push_back(marker_left);
        marker_array.markers.push_back(marker_right);
        
        y -= y_step;
        i++;
    }
    
    tf::TransformListener tf;
    ros::Rate loop_rate(ROS_RATE);
    
    geometry_msgs::Point current_goal = points.front();
    points.pop();
    
    i = 0;
    while (ros::ok()){
        try{    
            tf.lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            continue;
        }
        
        float robot_x = robot_pose.getOrigin().x();
        float robot_y = robot_pose.getOrigin().y();
        //Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        //tf::Vector3 robot_axis=robot_pose.getRotation().getAxis();
        //double robot_theta=robot_pose.getRotation().getAngle()*robot_axis[2] * 180 / M_PI; // only need the z axis
        
        std::cout << "Robot Position: {x:" << robot_x << ", y: " << robot_y << "}" << std::endl; // << " Angle: " << robot_pose.getRotation().getAngle()*robot_axis[2] * 180 / M_PI << " Normalized angle: " << normalize_angle(robot_theta) << std::endl;
        std::cout << "Current Goal: {x:" << current_goal.x << ", y: " << current_goal.y << "}" << std::endl;
        
        if(are_two_points_close(robot_x, robot_y, current_goal.x,  current_goal.y)){
            if(points.empty()){
                std::cout << "## Circle traversed" << std::endl;
                break;
            }else{
                current_goal = points.front();
                points.pop();
            }
        }

        geometry_msgs::PointStamped current_point_stamped;
        current_point_stamped.header.frame_id = "/world";
        current_point_stamped.header.stamp = ros::Time();
        current_point_stamped.header.seq = i++;
        current_point_stamped.point.x = current_goal.x;
        current_point_stamped.point.y = current_goal.y;
        
        waypoint_publisher.publish(current_point_stamped);
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

float normalize_angle(float angle_in_degree){
    while(angle_in_degree > 360){
        angle_in_degree -= 360;
    }
    while(angle_in_degree < -360){
        angle_in_degree += 360;
    }
    if(angle_in_degree > 180){
        angle_in_degree -= 360;
    }else if(angle_in_degree < -180){
        angle_in_degree += 360;
    }
    return angle_in_degree;
}

bool are_two_points_close(double x_1, double y_1, double x_2, double y_2){
    return are_two_points_close(x_1, x_2, y_1, y_2, PROXIMITY_THERESHOLD);
}

bool are_two_points_close(double x_1, double y_1, double x_2, double y_2, double threshold){
    if(distance_between_two_points(x_1, x_2, y_1, y_2) > threshold)
        return false;
    return true;
}
double distance_between_two_points(double x_1, double y_1, double x_2, double y_2){
    return sqrt((x_1 - x_2)*(x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
}