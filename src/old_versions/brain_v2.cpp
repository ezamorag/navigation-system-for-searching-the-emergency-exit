#include <ros/ros.h>
#include "find_emergency_exit/detect_sign.h"
#include "find_emergency_exit/motion_goal.h"
#include "find_emergency_exit/search_sign.h"
ros::ServiceClient cdetect_sign, cmotion_goal;

bool search_sign_cb(find_emergency_exit::search_sign::Request  &req, find_emergency_exit::search_sign::Response &res) {
   find_emergency_exit::detect_sign srvDS;
   find_emergency_exit::motion_goal srvMG;

   // Stop the robot, there is a new sign to detect
   srvMG.request.x = NAN;
   srvMG.request.y = NAN;
   srvMG.request.th = NAN;
   if (!cmotion_goal.call(srvMG)) { ROS_ERROR("Failed to call service motion_goal"); return 1; }
   if (srvMG.response.isDone) { ROS_INFO("The goal was set"); 
   } else { ROS_INFO("Something failed"); }

   // Sign Detection
   if (!cdetect_sign.call(srvDS)) { ROS_ERROR("Failed to call service DetectSign"); return 1; }
   ROS_INFO("x: %f", srvDS.response.x);
   ROS_INFO("y: %f", srvDS.response.y);
   ROS_INFO("th: %f", srvDS.response.th);

   // Move the robot according to the new sign
   srvMG.request.x = srvDS.response.x;
   srvMG.request.y = srvDS.response.y;
   srvMG.request.th = srvDS.response.th;
   if (!cmotion_goal.call(srvMG)) { ROS_ERROR("Failed to call service motion_goal"); return 1; }
   if (srvMG.response.isDone) { ROS_INFO("The goal was set"); 
   } else { ROS_INFO("Something failed"); }
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "brain");
   ros::NodeHandle node;
   cdetect_sign = node.serviceClient<find_emergency_exit::detect_sign>("detect_sign");
   cmotion_goal = node.serviceClient<find_emergency_exit::motion_goal>("motion_goal");
   ros::ServiceServer search_sign_srv = node.advertiseService("search_sign", &search_sign_cb);
   ros::spin();
   return 0;
}
