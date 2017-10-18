#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core/core.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>

#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <fstream>

#include <snd_mike/SensorPacket.h>
#include "find_emergency_exit/motion_goal.h"

using namespace cv;
using namespace std;

// Sigue valleys fantamas (cuando está encerrado por paredes, sigue valleys falsos)
// Genera vibraciones 
// 
// vs (Mapa local + Planeamiento + Controlador) 

/*** Global Variables and Parameters***/
double Dir, DirNow, DirGoal, xs=NAN, ys=NAN, ths=NAN;
geometry_msgs::Twist nav;
ros::Publisher cmdvelPub, direction_pub_;
int Ns;
const double rangeTh = M_PI/3.0, maxV=0.22, maxW=0.3, R=0.3, Ds=1.0, GapDist = 1.0; 
/* Functions */
double distc(double alpha, double beta) { return fmod(alpha-beta,2*M_PI);}
double distcc(double alpha, double beta) { return fmod(beta-alpha,2*M_PI);}
double dist(double alpha, double beta) { return fmin(distc(alpha,beta),distcc(alpha,beta)); }
//double proj(double theta) { return fmod(theta+M_PI,2*M_PI)-M_PI;}
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
double bisec(double s) { return rangeTh/2.0*(1.0-2.0*s/(Ns-1)); } //Cuidado apesar que funciona me parece que el signo es incorrecto, es decir, el laser gira en el otro sentido

bool motion_goal_cb(find_emergency_exit::motion_goal::Request  &req, find_emergency_exit::motion_goal::Response &res) {
   xs = req.x;
   ys = req.y;
   ths = proj(req.th);
   ROS_INFO("xs: %f, ys: %f, ths: %f",xs,ys,ths);
   res.isDone = true;
   return true;
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

double directionNav(double x, double y){
   double th2 = ths + 3.0/4.0*M_PI;
   double a = cos(th2) + sin(th2);
   double b = sin(th2) - cos(th2);
   double c = -a*xs - b*ys;
   double darrow = (a*x + b*y + c)/sqrt(a*a + b*b);
   if (darrow > 0.25) return proj(ths - M_PI/4.0);
   else if (darrow < -0.25) return proj(ths + M_PI/4.0);
   else return proj(ths);
}

void odomcallback(const nav_msgs::Odometry::ConstPtr& currodom) { //(~14Hz)
   if ((isnan(xs)) && (isnan(ys)) && (isnan(ths))) { return; }
   //Compute actual direction  
   tf::Pose pose;
   tf::poseMsgToTF(currodom->pose.pose, pose);
   DirNow = tf::getYaw(pose.getRotation());
   Dir = directionNav(currodom->pose.pose.position.x, currodom->pose.pose.position.y);
   DirGoal = proj(Dir - DirNow); // DirNow and Dir are bounded between -pi and pi
}
 
void snd_cb(const sensor_msgs::LaserScan::ConstPtr& scan_msg) { //(~30Hz)

   if ((isnan(xs)) && (isnan(ys)) && (isnan(ths))) {
      ROS_INFO("NaN values in (xs,ys,ths), robot stops");
      nav.linear.x = 0.0;  
      nav.angular.z = 0.0;
      cmdvelPub.publish(nav);
      return;
   }

   

   double DirSND;
   Ns = scan_msg->ranges.size();
   ROS_DEBUG("Number of beams: %d",scan_msg->ranges.size());   
   
   // Check the laser data is clean from NaN values
   vector<double> scan;
   for(int ith=0; ith<Ns; ith++) {
      if (isnan(scan_msg->ranges[ith])) {
         ROS_ERROR("There are NaN values in laser data");
         return;
      }
      else
         scan.push_back(scan_msg->ranges[ith]);    
   }
   
   // Delete isolated NaN values 
   for(int ith=1; ith<scan.size()-1; ith++) {
      if ( (scan[ith-1]<= scan_msg->range_max) && (scan[ith] > scan_msg->range_max) && (scan[ith+1]<= scan_msg->range_max) ) {
         scan[ith] = (scan[ith-1] + scan[ith+1])/2.0;
      }
   }
   
   
   // Finding the gaps
   double drange;
   vector<int> gaps, gapsLR;
   if (scan[0] > scan_msg->range_max) {
      gaps.push_back(0);   // It is always a left gap 
      gapsLR.push_back(1);
   }
   for(int ith=1; ith<Ns-1; ith++) { 
      drange = scan[ith+1]-scan[ith];
      if (abs(drange) > GapDist) {
         if (drange > 0) {
            gaps.push_back(ith);
            gapsLR.push_back(1);  // Left  gap
         } else {
            gaps.push_back(ith+1);
            gapsLR.push_back(2);  // Right gap
         }
      }        
   }
   if (scan[Ns-1] > scan_msg->range_max) {
      gaps.push_back(Ns-1);   // It is always a right gap 
      gapsLR.push_back(2); 
   } 

   if (gaps.size() == 0) {   //The robot is enclosed by walls, no gaps
      ROS_ERROR("No Gaps");
      nav.linear.x = 0;  
      if (DirGoal > 0) {
         nav.angular.z = maxW;
         DirSND = M_PI/6.0;
      } else {
         nav.angular.z = -maxW;
         DirSND = -M_PI/6.0;
      }
      cmdvelPub.publish(nav);

      cout << "dirGoal (sensor): "<< DirGoal*180/M_PI << endl;
      cout << "dirSND (sensor): "<< DirSND*180/M_PI << endl;
      cout << "dirNow (world): "<< DirNow*180/M_PI << endl;
      cout << "Dir (world): "<< Dir*180/M_PI << endl;
      cout << "Linear vel: "<< nav.linear.x << endl;
      cout << "Angular vel:  "<< nav.angular.z << endl;


      // Compute the pose (position, orientation) of plane
      geometry_msgs::PoseStamped direction;
      direction.header.stamp = ros::Time::now();
      direction.header.frame_id = scan_msg->header.frame_id;   
      direction.pose.position.x = 0.0; 
      direction.pose.position.y = 0.0;
      direction.pose.position.z = 0.0;      
      direction.pose.orientation.x = 0.0;
      direction.pose.orientation.y = 0.0;
      direction.pose.orientation.z = sin(DirSND/2);
      direction.pose.orientation.w = cos(DirSND/2);
      direction_pub_.publish(direction);
   
      return;
   }
   
   // Finding the valleys and the best valley
   std::vector<std::vector<double> > valleys;
   vector<int> flagvalleys;
   flagvalleys.assign (gaps.size(),0);
   int ithrg;
   double Th1, Th2, Thrg, Thog, LR_tmp, Drg_tmp;
   for(int ig=0; ig<gaps.size()-1; ig++) {
      if ((gapsLR[ig] == 1) && (gapsLR[ig+1] == 2)) {
         flagvalleys[ig] = 1;
         flagvalleys[ig+1] = 1;
         Th1 = bisec(gaps[ig]);
         Th2 = bisec(gaps[ig+1]);
         if (abs(proj(DirGoal - Th1)) <= abs(proj(DirGoal - Th2))) { 
            Thrg = Th1;
            Thog = Th2;
            ithrg = 0;
            if ((gaps[ig] == 0) || (gaps[ig] == Ns-1))
               Drg_tmp = 100000.0;   // The area is completely open
            else {
               if (scan[gaps[ig]] <= scan[gaps[ig]+1])
                  Drg_tmp = scan[gaps[ig]];
               else 
                  Drg_tmp = scan[gaps[ig]+1];
            }
            LR_tmp = gapsLR[ig];
         } else {
            Thrg = Th2;
            Thog = Th1;
            ithrg = 1;
            if ((gaps[ig+1] == 0) || (gaps[ig+1] == Ns-1))
               Drg_tmp = 100000.0;   // The area is completely open
            else {
               if (scan[gaps[ig+1]] <= scan[gaps[ig+1]+1])
                  Drg_tmp = scan[gaps[ig+1]];
               else 
                  Drg_tmp = scan[gaps[ig+1]+1];
            }
            LR_tmp = gapsLR[ig+1];
         }
         std::vector<double> tmp;
         tmp.push_back(gaps[ig]);
         tmp.push_back(gaps[ig+1]);
         tmp.push_back(Th1);
         tmp.push_back(Th2);
         tmp.push_back(Thrg);
         tmp.push_back(Thog);
         tmp.push_back(LR_tmp);
         tmp.push_back(Drg_tmp);
         tmp.push_back(ithrg);
         valleys.push_back(tmp);  
      }
   }
   // Gaps in isolation as valleys (can be possible free routes)
   double Th3;
   for(int ig=0; ig<gaps.size(); ig++) {
      if (flagvalleys[ig] == 0){
         Th3 = bisec(gaps[ig]);
         std::vector<double> tmp1;
         tmp1.push_back(gaps[ig]);        //gapLeft
         tmp1.push_back(gaps[ig]);        //gapRight
         tmp1.push_back(Th3);             //Th1
         tmp1.push_back(Th3);             //Th2
         tmp1.push_back(Th3);             //Thrg
         tmp1.push_back(Th3);             //Thog
         tmp1.push_back(gapsLR[ig]);      //LR
         tmp1.push_back(scan[gaps[ig]]);  //Drg
         tmp1.push_back(0);               //ithrg
         valleys.push_back(tmp1);
      }
   }
   // Best valley
   int ibest = -1, ibest_ithrg;
   double minDist = 2*M_PI, Trg, Tog, LR, Drg;
   for(int iv=0; iv<valleys.size(); iv++) {
      Thrg = valleys[iv][4];
      if (abs(proj(DirGoal - Thrg)) < minDist) {
         minDist = abs(proj(DirGoal - Thrg));
         Trg = Thrg;
         Tog = valleys[iv][5];
         LR = valleys[iv][6]; 
         Drg = valleys[iv][7];
         ibest = iv;
         ibest_ithrg = valleys[iv][8];
      }
   }
   
   if (minDist == 2*M_PI) { 
      ROS_ERROR("No Valleys");
      return; 
   } else {
      //cout << "Valleys: " << valleys << endl;
      //cout << "BestValley: " << ibest << endl;
   }

   //Computing desire angle
   double theta1 = valleys[ibest][2];   //Best Valley limits 
   double theta2 = valleys[ibest][3];
   //cout << "Drg = " << Drg << endl;
   //cout << "Trg = " << Trg*180/M_PI << endl;
   if (DirGoal>theta2 && DirGoal<theta1) {   //Always th1 >= th2
      ROS_ERROR("TGoal"); DirSND=DirGoal; 
   } else {
      double Tsrg;
      if (Drg<R+Ds) Drg = R+Ds;              // Para evitar generar NaN cuando (R+Ds)/Drg>1
      if (LR == 1) Tsrg=Trg-asin((R+Ds)/Drg);   
      else Tsrg=Trg+asin((R+Ds)/Drg);

      double Tmid;
      if (LR == 1) Tmid=Trg-distc(Trg,Tog)/2; 
      else Tmid=Trg+distcc(Trg,Tog)/2; 
      
     // cout << "Tsrg = " << Tsrg*180/M_PI << endl;
     // cout << "Tmid = " << Tmid*180/M_PI << endl;
   
      if (abs(dist(Trg,Tmid))<abs(dist(Trg,Tsrg))) { 
         ROS_ERROR("Tmid"); DirSND=Tmid;
      } else { 
         ROS_ERROR("Tsrg"); DirSND=Tsrg;
      }
   }

   // Obstacle Avoidance method
   vector<double> s, deltaI;
   s.assign (Ns,0.0);
   deltaI.assign (Ns,0.0);
   int Nmid = round(Ns/2.0);
   double sTot = 0.0, sTleft=0.0, sTright=0.0;
   for (int x=0; x<Ns; x++) {  
      s[x] = sat(0,1,(Ds+R-(scan[x]-scan_msg->range_min))/Ds);   
      deltaI[x] = s[x]*proj(distcc(M_PI/2 + bisec(x),DirSND));
      if (x <= Nmid)
         sTleft += s[x]*s[x];
      else
         sTright += s[x]*s[x];
   }
   sTot = sTleft + sTright;
   double DeltaAvoid=0.0; 
   if (sTot==0) {DeltaAvoid=0.0;}
   else {
      for (int x=0; x<Ns; x++) 
         DeltaAvoid += (s[x]*s[x]*deltaI[x])/sTot;
   }
   if (sTleft < sTright)    //porque no funciona sin esto?
      DirSND += DeltaAvoid;
   else
      DirSND -= DeltaAvoid;
   cout << "delta all: "<< DeltaAvoid*180/M_PI << endl;
   
   nav.linear.x = maxV*sat(0.0,1.0, (M_PI/4.0 - abs(DirSND))/(M_PI/4.0));  //Modula que tanto girar y trasladarse
   nav.angular.z = maxW*sat(-1.0,1.0, DirSND/(M_PI/6.0));
   cmdvelPub.publish(nav); 

   cout << "dirGoal (sensor): "<< DirGoal*180/M_PI << endl;
   cout << "dirSND (sensor): "<< DirSND*180/M_PI << endl;
   cout << "dirNow (world): "<< DirNow*180/M_PI << endl;
   cout << "Dir (world): "<< Dir*180/M_PI << endl;
   cout << "Linear vel: "<< nav.linear.x << endl;
   cout << "Angular vel:  "<< nav.angular.z << endl << endl;
   

   // Compute the pose (position, orientation) of plane
   geometry_msgs::PoseStamped direction;
   direction.header.stamp = ros::Time::now();
   direction.header.frame_id = scan_msg->header.frame_id;   
   direction.pose.position.x = 0.0; 
   direction.pose.position.y = 0.0;
   direction.pose.position.z = 0.0;      
   direction.pose.orientation.x = 0.0;
   direction.pose.orientation.y = 0.0;
   direction.pose.orientation.z = sin(DirSND/2);
   direction.pose.orientation.w = cos(DirSND/2);
   direction_pub_.publish(direction);
  
}

int main (int argc, char** argv) {
  ros::init (argc, argv, "SND_Navigation");
  ros::NodeHandle nh;
  ROS_ERROR("Navigating... START");

  ros::Subscriber odom_sub = nh.subscribe("/odom", 1, &odomcallback);
  ros::Subscriber scan_sub = nh.subscribe("/base_scan", 1, &snd_cb);
  ros::Subscriber sensorPacket_sub = nh.subscribe("/sensorPacket", 1, &sensorPacket_cb);

  cmdvelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  direction_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/direction", 1);
  ros::ServiceServer srvdetections = nh.advertiseService("motion_goal", &motion_goal_cb);

  ros::spin();
  return 0;
}
