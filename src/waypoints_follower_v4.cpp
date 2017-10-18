#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <find_emergency_exit/SensorPacket.h>
#include <find_emergency_exit/on_off.h>

using namespace std;

/*** Global Variables and Parameters***/
std::vector<std::vector<double> > waypoints;
ros::Publisher cmdvelPub;
geometry_msgs::Twist nav;
const double maxV=0.2, maxW=0.8, c1 = M_PI/6.0;    //maxV < 0.4 to avoid an abrupt stop and maxV > 0.3 to have good odometry  0.3 y 1.0
const double tolDist = 0.1;        // tolDist > 0.06 to avoid oscillations 
bool flagStop = true;
int past_seq = 10000000, i_current; 
double DirGoal; 
/* Functions */
//double proj(double theta) { return fmod(theta+M_PI,2*M_PI)-M_PI;}  // Esta ecuación se equivoca para valores negativos de Theta!!!!
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

bool stop_cb(find_emergency_exit::on_off::Request  &req, find_emergency_exit::on_off::Response &res) {
     flagStop = req.switch1;
     res.isDone = true;
     return true;
  }

void sensorPacket_cb(const find_emergency_exit::SensorPacket::ConstPtr& sensorPacket_msg_cb) {
   
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


void waypoints_cb(const nav_msgs::Path::ConstPtr& waypoints_cb) {
      // WATCH OUT: The planner must publish the waypoints only when there is a new plan
      // Otherwise, the robot oscillates around the first waypoint

      if (waypoints_cb->header.seq != past_seq) {   // If there is a new path plan, change the waypoints
         past_seq = waypoints_cb->header.seq;
         waypoints.clear();
         for (int i=0; i<waypoints_cb->poses.size(); i++) {
            std::vector<double> point(2);
            point[0] = waypoints_cb->poses[i].pose.position.x;
            point[1] = waypoints_cb->poses[i].pose.position.y;
            waypoints.push_back(point);
         }
         i_current = 0;
      }
   }

void robot_localization_cb(const geometry_msgs::PoseStamped::ConstPtr& robot) {
   if ((flagStop) || (waypoints.size() == 0) ) {
      //ROS_ERROR("Robot stops");
      nav.linear.x = 0.0;  
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      return;
   }
   
   //Compute actual direction 
   double roll, pitch;
   double Nowx, Nowy, WPLength, DirNow;
   tf::Pose pose;
   tf::poseMsgToTF(robot->pose, pose);
   DirNow = tf::getYaw(pose.getRotation());
   Nowx=robot->pose.position.x; 
   Nowy=robot->pose.position.y;
   
   
   double Goalx = waypoints[i_current][0];
   double Goaly = waypoints[i_current][1];
   WPLength=sqrt((Goaly-Nowy)*(Goaly-Nowy)+(Goalx-Nowx)*(Goalx-Nowx)); 
   DirGoal = proj(atan2(Goaly-Nowy, Goalx-Nowx) - DirNow); 

   
   if (WPLength < tolDist) {
      nav.linear.x = 0.0;
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      i_current++;
      if (i_current > waypoints.size()-1)  i_current = waypoints.size()-1; 
      return;
   }

   nav.linear.x = maxV*sat(0.0,1.0, (c1 - abs(DirGoal))/c1);  
   nav.angular.z = maxW*sat(-1.0,1.0, DirGoal/c1);
   cmdvelPub.publish(nav); 

   cout << "dirGoal (sensor): "<< DirGoal*180/M_PI << endl;
   cout << "dirNow (world): "<< DirNow*180/M_PI << endl;
   cout << "Linear vel: "<< nav.linear.x << endl;
   cout << "Angular vel:  "<< nav.angular.z << endl;
   cout << "x: " << Nowx << endl;
   cout << "y: " << Nowy << endl;
}

int main (int argc, char** argv) {
  ros::init (argc, argv, "waypoints_follower");
  ros::NodeHandle nh;
  ROS_INFO("Waiting for waypoints plan");


  ros::Subscriber waypoints_sub = nh.subscribe("/waypoints", 1, &waypoints_cb);
  ros::Subscriber loc_sub = nh.subscribe("/robotpose", 1, &robot_localization_cb);
  ros::Subscriber sensorPacket_sub = nh.subscribe("/sensorPacket", 1, &sensorPacket_cb);
  cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  ros::ServiceServer srv_stop = nh.advertiseService("stop_robot", &stop_cb);
  ros::spin();
  return 0;
}
