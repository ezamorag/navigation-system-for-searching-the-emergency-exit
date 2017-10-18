#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
using namespace std;

/*** Global Variables and Parameters***/
geometry_msgs::Twist nav;
ros::Publisher cmdvelPub;

void odomcallback(const nav_msgs::Odometry::ConstPtr& currodom) { //(~14Hz) 
   tf::Pose pose;
   tf::poseMsgToTF(currodom->pose.pose, pose);
   double DirNow = tf::getYaw(pose.getRotation());
   nav.linear.x = 0.0;  
   nav.angular.z = 1.0;
   cmdvelPub.publish(nav); 
   cout << "Theta (world): "<< DirNow*180/M_PI << endl;
   cout << "Linear vel: "<< nav.linear.x << endl;
   cout << "Angular vel:  "<< nav.angular.z << endl << endl;
}

int main (int argc, char** argv) {
  ros::init (argc, argv, "errorodom_test");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomcallback);
  cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::spin();
  return 0;
}
