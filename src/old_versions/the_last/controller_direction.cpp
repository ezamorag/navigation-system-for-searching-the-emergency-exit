#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <snd_mike/SensorPacket.h>
#include "find_emergency_exit/motion_goal.h"

using namespace std;

/*** Global Variables and Parameters***/
double DirGoal, xs=NAN, ys=NAN, ths=NAN;
geometry_msgs::Twist nav;
ros::Publisher cmdvelPub, direction_pub_;
const double maxV=0.3, maxW=1.0, c1 = M_PI/6.0;    // WATCH OUT: When the robot turns slowly, the odometry fails systematically

//double proj(double theta) { return fmod(theta+M_PI,2*M_PI)-M_PI;}  // Equivocada!!
double proj(double theta) { 
   double newtheta = fmod(theta,2*M_PI);
   if (newtheta < -M_PI) return newtheta+2*M_PI;
   else if (newtheta > M_PI) return newtheta-2*M_PI;
   else return newtheta; 
}
double sat(int a, int b, double x) { 
   if (x<=a) x=a; 
   else if (x>=b) x=b;
   else x=x; 
   return x;
}

bool motion_goal_cb(find_emergency_exit::motion_goal::Request  &req, find_emergency_exit::motion_goal::Response &res) {
   xs = req.x;
   ys = req.y;
   ths = proj(req.th);
   ROS_INFO("xs: %f, ys: %f, ths: %f",xs,ys,ths);
   res.isDone = true;
   return true;
}


double directionNav(double x, double y){   //(x,y) odometry
   double th2 = ths + 3.0/4.0*M_PI;
   double a = cos(th2) + sin(th2);
   double b = sin(th2) - cos(th2);
   double c = -a*xs - b*ys;
   double darrow = (a*x + b*y + c)/sqrt(a*a + b*b);
   if (darrow > 0.25) return proj(ths - M_PI/4.0);
   else if (darrow < -0.25) return proj(ths + M_PI/4.0);
   else return proj(ths);
}

void sensorPacket_cb(const snd_mike::SensorPacket::ConstPtr& sensorPacket_msg_cb) {
   
   if ( ((sensorPacket_msg_cb->bumpLeft == true) || (sensorPacket_msg_cb->bumpRight == true))) {
      // Go back
      nav.linear.x = -0.2;  
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      ros::Duration(1.0).sleep();
      if ((sensorPacket_msg_cb->bumpLeft == true) && (sensorPacket_msg_cb->bumpRight == true)) {
         ROS_ERROR("Both bumpers");
         //Turn to the right or left depending on goal direction
         nav.linear.x = 0.0; 
         if (DirGoal >= 0.0) nav.angular.z = 0.6; 
         else nav.angular.z = -0.6;   
         cmdvelPub.publish(nav);
         ros::Duration(2.0).sleep();  
      } else if (sensorPacket_msg_cb->bumpRight == true) {
         ROS_ERROR("Right bumper");
         // Turn to the left a bit
         nav.linear.x = 0.0;  
         nav.angular.z = 0.6;
         cmdvelPub.publish(nav);
         ros::Duration(1.0).sleep();
      } else {
         ROS_ERROR("Left bumper");
         // Turn to the right a bit
         nav.linear.x = 0.0;  
         nav.angular.z = -0.6;
         cmdvelPub.publish(nav);
         ros::Duration(1.0).sleep();
      }
      // Go forward
      nav.linear.x = 0.2;  
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      ros::Duration(1.0).sleep();
   }
}

void odomcallback(const nav_msgs::Odometry::ConstPtr& currodom) { //(~14Hz)
   if ((isnan(xs)) && (isnan(ys)) && (isnan(ths))) {
      ROS_INFO("NaN values in (xs,ys,ths), robot stops");
      nav.linear.x = 0.0;  
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      return;
   }

   //Compute actual direction  
   tf::Pose pose;
   tf::poseMsgToTF(currodom->pose.pose, pose);
   double DirNow = tf::getYaw(pose.getRotation());
   double Dir = directionNav(currodom->pose.pose.position.x, currodom->pose.pose.position.y);

   DirGoal = proj(Dir - DirNow);   // DirNow and Dir are bounded between -pi and pi

   nav.linear.x = maxV*sat(0.0,1.0, (c1 - abs(DirGoal))/c1);  
   nav.angular.z = maxW*sat(-1.0,1.0, DirGoal/c1);
   cmdvelPub.publish(nav); 

   ROS_DEBUG("dirGoal (sensor, °): %.2f", DirGoal*180/M_PI);
   ROS_DEBUG("dirNow (world, °): %.2f", DirNow*180/M_PI); 
   ROS_DEBUG("Dir (world, °): %.2f", Dir*180/M_PI);
   ROS_DEBUG("Linear vel: %.2f", nav.linear.x);
   ROS_DEBUG("Angular vel:  %.2f", nav.angular.z);
   
   // Compute the pose 
   geometry_msgs::PoseStamped direction;
   direction.header.stamp = ros::Time::now();
   direction.header.frame_id = "base_laser";   
   direction.pose.position.x = 0.0; 
   direction.pose.position.y = 0.0;
   direction.pose.position.z = 0.0;      
   direction.pose.orientation.x = 0.0;
   direction.pose.orientation.y = 0.0;
   direction.pose.orientation.z = sin(DirGoal/2);
   direction.pose.orientation.w = cos(DirGoal/2);
   direction_pub_.publish(direction);
}

int main (int argc, char** argv) {
  ros::init (argc, argv, "controller_direction");
  ros::NodeHandle nh;

  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomcallback);
  ros::Subscriber sensorPacket_sub = nh.subscribe("/sensorPacket", 1, &sensorPacket_cb);

  cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  direction_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/direction", 1);
  ros::ServiceServer srvdetections = nh.advertiseService("motion_goal", &motion_goal_cb);

  ros::spin();
  return 0;
}
