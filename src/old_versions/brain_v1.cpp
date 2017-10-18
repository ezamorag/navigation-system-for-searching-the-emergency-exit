#include <ros/ros.h>
#include "find_emergency_exit/detect_sign.h"
#include "find_emergency_exit/motion_goal.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "brain");
  ros::NodeHandle n;
  ros::ServiceClient cdetect_sign = n.serviceClient<find_emergency_exit::detect_sign>("detect_sign");
  ros::ServiceClient cmotion_goal = n.serviceClient<find_emergency_exit::motion_goal>("motion_goal");
  find_emergency_exit::detect_sign srvDS;
  find_emergency_exit::motion_goal srvMG;

  // Detect the sign
  if (!cdetect_sign.call(srvDS)) { ROS_ERROR("Failed to call service detect_sign"); return 1; }
  ROS_INFO("x: %f", srvDS.response.x);
  ROS_INFO("y: %f", srvDS.response.y);
  ROS_INFO("th: %f", srvDS.response.th);

  // Move the robot according to the sign
  srvMG.request.x = srvDS.response.x;
  srvMG.request.y = srvDS.response.y;
  srvMG.request.th = srvDS.response.th;
  if (!cmotion_goal.call(srvMG)) { ROS_ERROR("Failed to call service motion_goal"); return 1; }
  if (srvMG.response.isDone) { ROS_INFO("The goal was set"); 
  } else { ROS_INFO("Something failed"); }

  return 0;
}
