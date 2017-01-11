# kubot

Path planning and object/color detecting hector quadrator for Istanbul Technical University Robotics course project.

**for launching**

    roslaunch kubot_launch kubot_traverse_test.launch
   
   **for starting circle traversal waypoint publisher**
   
    rosrun kubot_circle_traverse kubot_circle_traverse 
    
   **for starting PointCloud subscriber node for object/color detection**
   
    rosrun point_cloud_package pointcloud_subs 
  
  **Finally, for path planner**
  
    rosrun kubot_planner kubot_planner
