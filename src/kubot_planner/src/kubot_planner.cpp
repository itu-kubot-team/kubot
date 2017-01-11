#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>  
#include <cmath>

#define PI 3.14159265
#define MIN_ROTATION_SPEED 0.2
#define MAX_ROTATION_SPEED 0.3
#define MAX_ANGLE_DIFF  180
#define MIN_LINEAR_SPEED 0.1
#define MAX_LINEAR_SPEED 0.6
#define ROBOT_FLYING_HEIGHT 1.5

float marker_counter = 0;
float dir_x;
float dir_y;
float goal_x = 0;
float goal_y = 0;
bool stopGettingWaypointInfo = false;
ros::Publisher marker_publisher, motor_command_publisher;
ros::Subscriber map_subscriber, waypoint_subscriber, laserscan_subscriber, pointcloud_subscriber;
tf::StampedTransform robot_pose;
geometry_msgs::Twist motor_command;
visualization_msgs::MarkerArray marker_array;
tf::TransformListener* tListener;
visualization_msgs::Marker current_marker;
sensor_msgs::LaserScan laser_msg;

bool isGoalSet = false;

float* potential_field_goal(float x, float y){
    float _dir_x = goal_x - x;
    float _dir_y = goal_y - y;
    float *pf_goal = new float[2];
    pf_goal[0] = _dir_x;
    pf_goal[1] = _dir_y;
    return pf_goal;
}

float* potential_field_obst(float x,float y, nav_msgs::OccupancyGrid grid){
    float originx = grid.info.origin.position.x;
    float originy = grid.info.origin.position.y;
    
    float min_dist = 9999999999;
    float min_x;
    float min_y;
    float _dir_x;
    float _dir_y;
    float *pf_obst = new float[2];
    
    /* iterate over map grid information */
    for(int X=0; X < grid.info.width; X++){
        for(int Y=0; Y < grid.info.width; Y++){
            
            float obst_x = originx + X*grid.info.resolution;
            float obst_y = originy + Y*grid.info.resolution;
            float occupancy = grid.data[Y*grid.info.width + X]/100; /* probability of occupancy returns in % 0-100 */
       
            if (occupancy<0) 
                occupancy=1.0;
            
            /* if grid is occupied with %100 probability or occupancy information is not known. */
            if(occupancy > 0.99){
                float dist = sqrt(pow(obst_x - x,2) + pow(obst_y - y, 2));
                if(dist < min_dist){
                    min_dist = dist;
                    min_x = obst_x;
                    min_y = obst_y;
                }
            }
        }
    }
    
    if(min_dist == 0){
        _dir_x = 0;
        _dir_y = 0;
    }
    else {
        _dir_x = (x-min_x) / pow(min_dist,4);
        _dir_y = (y-min_y) / pow(min_dist,4);
    }
    
    pf_obst[0] = _dir_x * 1;
    pf_obst[1] = _dir_y * 1;
    return pf_obst;
}

void potential_field(float x,float y,float goal_x, float goal_y, nav_msgs::OccupancyGrid grid){
    float *pf_goal = potential_field_goal(x,y);
    float *pf_obst = potential_field_obst(x,y,grid);
    dir_x = pf_goal[0] + pf_obst[0];
    dir_y = pf_goal[1] + pf_obst[1];
}

visualization_msgs::Marker arrow_marker_at(float x, float y) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/world";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    geometry_msgs::Point tail;
    tail.x = x;
    tail.y = y;
    tail.z = robot_pose.getOrigin().z();
    geometry_msgs::Point head;
    head.x = x+dir_x*0.1;
    head.y = y+dir_y*0.1;
    head.z = robot_pose.getOrigin().z();
    
    marker.points.push_back(tail);
    marker.points.push_back(head);
    
    marker.scale.x = 0.02;
    marker.scale.y = 0.08;
    marker.scale.z = 0;
    
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.id = marker_counter++;
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


void setGoalMarker(){    
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "/world";
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.id = marker_counter++;
    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    goal_marker.pose.position.z = robot_pose.getOrigin().z();
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.1;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;
    marker_publisher.publish(goal_marker);
}

void map_callback(nav_msgs::OccupancyGrid grid){
    tListener->lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
    std::cout << "Robot: x: " << robot_pose.getOrigin().x() << " y: " << robot_pose.getOrigin().x() << std::endl;
    std::cout << "Goal: x: " << goal_x << " y: " << goal_y << std::endl;
    
    potential_field(robot_pose.getOrigin().x(), robot_pose.getOrigin().y(), goal_x, goal_y, grid);
    visualization_msgs::Marker marker = arrow_marker_at(robot_pose.getOrigin().x(),robot_pose.getOrigin().y());
    marker_array.markers.push_back(marker);
    marker_publisher.publish(marker);
    current_marker = marker;
}

void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr& waypoint_in){
    if(stopGettingWaypointInfo != true){ /* if object is not detected yet, get new goals */
        goal_x = waypoint_in->point.x;
        goal_y = waypoint_in->point.y;
        isGoalSet = true;
    }
}

void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    laser_msg = *msg;
}

void pointcloud_callback(std_msgs::String detection_info){
    if(detection_info.data == "found red"){
        stopGettingWaypointInfo  = true; /* to prevent getting new goals from waypoint */
        std::cout << "Found red sphere!" << std::endl;
        goal_x = 0;
        goal_y = 0;
        setGoalMarker();
    }
    
    else if(detection_info.data == "found blue"){
        stopGettingWaypointInfo  = true; /* to prevent getting new goals from waypoint */
        std::cout << "Found blue sphere!" << std::endl;
        goal_x = 0;
        goal_y = 0;
        setGoalMarker();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "Kubot_Planner");
    ros::NodeHandle n, m;
    tf::TransformListener listener;
    tListener = &listener;
    motor_command_publisher = m.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    marker_publisher = n.advertise<visualization_msgs::Marker>("/marker_vis", 1);
    
    ros::Duration(0.5).sleep();
    
    /* first we need to engage the motors and fly up to 1.5 meters */
    motor_command.linear.x = 0.0;
    motor_command.linear.y = 0.0;
    motor_command.linear.z = 0.15;
    
    motor_command.angular.x = 0.0;
    motor_command.angular.y = 0.0;
    motor_command.angular.z = 0.0;
    motor_command_publisher.publish(motor_command);
    
    tListener->lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
    while(fabs(robot_pose.getOrigin().z() - ROBOT_FLYING_HEIGHT) > 0.01){
        tListener->lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
        std::cout << "height: " << robot_pose.getOrigin().z() << std::endl;
    }
    
    /* then we should stop and look for goals. */
    motor_command.linear.x = 0.0;
    motor_command.linear.y = 0.0;
    motor_command.linear.z = 0.0;
    
    motor_command.angular.x = 0.0;
    motor_command.angular.y = 0.0;
    motor_command.angular.z = 0.0;
    motor_command_publisher.publish(motor_command);
    
    map_subscriber = n.subscribe("/map", 1 , map_callback);
    waypoint_subscriber = n.subscribe("/waypoint", 1000, waypoint_callback);
    laserscan_subscriber = n.subscribe("/scan", 1000, laserscan_callback);
    pointcloud_subscriber = n.subscribe("/detectionresult", 100, pointcloud_callback);
    ros::Duration time_between_ros_wakeups(0.01);

    ros::Duration(1).sleep();
    
    
    while(ros::ok())
    {
        try{
            listener.waitForTransform("/world","/base_link", ros::Time(0), ros::Duration(10.0));
            listener.lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
        }
        catch (std::exception ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        ros::spinOnce();
        
        if(isGoalSet == false)
            continue;
        
        /* get head and tail of current arrow marker */
        geometry_msgs::Point tail = current_marker.points[0];
        geometry_msgs::Point head = current_marker.points[1];
        
        
        float marker_angle = normalize_angle(atan2(head.y - tail.y,head.x - tail.x) * 180/M_PI);
        tf::Vector3 robot_axis=robot_pose.getRotation().getAxis();
        double robot_theta= normalize_angle(robot_pose.getRotation().getAngle()*robot_axis[2] * 180/M_PI);
        double robot_theta_unnorm = robot_pose.getRotation().getAngle()*robot_axis[2] * 180 / M_PI;

        float angular_z = 0;
        float angle_dif = normalize_angle(robot_theta - marker_angle);
        if(fabs(angle_dif) > 5){ /* then i must change my angle in order to keep robot's camera vision looking forward*/
         
            float rot_speed = MIN_ROTATION_SPEED + (log(fabs(angle_dif))/log(MAX_ANGLE_DIFF)*(MAX_ROTATION_SPEED - MIN_ROTATION_SPEED));
            std::cout << "fabs olmayan difference: " << normalize_angle(robot_theta - marker_angle) << std::endl;
            if(normalize_angle(robot_theta - marker_angle) > 0){
                angular_z = -1* rot_speed;
            }
            else{
                angular_z = rot_speed;
            }
            motor_command.linear.x = 0;
            motor_command.linear.y = 0;
            motor_command.linear.z = 0.0;
                
            motor_command.angular.x = 0;
            motor_command.angular.y = 0;
            motor_command.angular.z = angular_z;
            motor_command_publisher.publish(motor_command);
                        
            ros::Duration(0.1).sleep();
            listener.lookupTransform("/world","/base_link", ros::Time(0), robot_pose);
            robot_theta= normalize_angle(robot_pose.getRotation().getAngle()*robot_axis[2] * 180/PI);
            std::cout << "normalize edilmemiş robot teta: " << robot_pose.getRotation().getAngle()*robot_axis[2] * 180/PI << " || " << robot_pose.getRotation().getAngle()*robot_axis[2] << std::endl;
            std::cout << "robot_theta : " << robot_theta << std::endl;
            std::cout << "marker_angle: " << marker_angle << std::endl;
        }
        
        else { /* then i can follow to current path */
            motor_command.linear.x = 0.30 * sqrt(pow(head.x - tail.x,2)+pow(head.y-tail.y,2));
            motor_command.linear.y = 0.0;
            motor_command.linear.z = 0.0;
            
            /* check forward, leftmost side and rightmost side. */
            /* if robot is too close */
            if(laser_msg.ranges[laser_msg.ranges.size() / 2] < 1 || (laser_msg.ranges[45*4] < 0.5) || (laser_msg.ranges[laser_msg.ranges.size()-45*4] < 0.5))
                motor_command.linear.x = 0.1;
            
            /* if robot is too fast */
            else if(laser_msg.ranges[laser_msg.ranges.size() / 2] > 10 && (laser_msg.ranges[laser_msg.ranges.size()-1] > 7.5) && (laser_msg.ranges[0] > 7.5))
                motor_command.linear.x *= 0.5 * laser_msg.ranges[laser_msg.ranges.size() / 2];
            
            /* if robot is too slow */
            if(fabs(motor_command.linear.x) < MIN_LINEAR_SPEED)
                motor_command.linear.x = MIN_LINEAR_SPEED;
            
            /* if robot is too fast */
            if(fabs(motor_command.linear.x) > MAX_LINEAR_SPEED)
                motor_command.linear.x = MAX_LINEAR_SPEED;

            motor_command.angular.x = 0;
            motor_command.angular.y = 0;
            motor_command.angular.z = 0.0;
            motor_command_publisher.publish(motor_command);
            std::cout << "Move: x: " << motor_command.linear.x << " y: " << motor_command.linear.y << std::endl;
        }
        
        time_between_ros_wakeups.sleep();
    }
    return 0;
}