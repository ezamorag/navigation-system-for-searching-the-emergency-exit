#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// Nota Importante: ros::Time(0) sirve para llamar el elemento del ultimo tiempo que exista
//                  ros::Time::now() calcula el tiempo actual de la computadora

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_localization");
  ros::NodeHandle node;
  ros::Publisher localization_pub = node.advertise<geometry_msgs::PoseStamped>("/robotpose", 1);
    
  tf::TransformListener listener;
  ros::Rate rate(20);   // Because the 
  while (node.ok()){
     tf::StampedTransform transform2, transform;
     try {
                                  //destination, source, 
        listener.waitForTransform("/odom","/base_link", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform2);          //
        listener.waitForTransform("/map","/odom", ros::Time(0), ros::Duration(3.0));
        listener.lookupTransform("/map", "/odom", ros::Time(0), transform);
     }
     catch (tf::TransformException ex)  { ROS_ERROR("%s",ex.what()); }
     transform*= transform2;   // The origin of base_link respect to map frame
     geometry_msgs::PoseStamped tmp;
     tmp.header.stamp = ros::Time::now();
     tmp.header.frame_id = "/map";
     tmp.pose.position.x = transform.getOrigin().x(); 
     tmp.pose.position.y = transform.getOrigin().y();
     tmp.pose.position.z = transform.getOrigin().z();
     tmp.pose.orientation.x = transform.getRotation().x();
     tmp.pose.orientation.y = transform.getRotation().y();
     tmp.pose.orientation.z = transform.getRotation().z();
     tmp.pose.orientation.w = transform.getRotation().w();
     localization_pub.publish(tmp);
     rate.sleep();
  }
  return 0;
}
