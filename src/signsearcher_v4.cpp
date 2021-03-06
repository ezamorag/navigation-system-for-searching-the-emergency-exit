/* 
    It detects the signs and find the reliable detections. It maintains a map of signs (x,y,th).
    It updates the goal when it's necesary and call the detection service when finds a new sign. 
*/
#include <ros/ros.h>
// OpenCV specific includes 
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
// PCL specific includes 
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
//#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "find_emergency_exit/signs_array.h"
#include "find_emergency_exit/motion_goal.h"
#include <find_emergency_exit/on_off.h>


int Nframes = 5, Nlandmarks = 20;
double r_pixelmax2 = 30*30, pframes = 1.0, drmaxc2 = 1.0*1.0;

class signsearcher {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_; 
  ros::Subscriber pcl_sub_, loc_sub_,tilt_sub_;
  ros::Publisher pcl_pub_, goal_pub_;
  cv::CascadeClassifier sign_cascade;
  std::vector<cv::Rect> signs;
  double th_tilt, xRobot, yRobot, thRobot;
  std::vector<std::vector<std::vector<double> > > bag, landmarks_bag;
  std::vector<std::vector<double> > rely_detections, ld;
  std::vector<double> arrow_directions;
  ros::ServiceClient csigns_array, cmotion_goal, cstop_planner, cstop_robot;

public:
  signsearcher()
    : it_(nh_) {   
     if( !sign_cascade.load("/home/erik/catkin_ws/src/signdetector/src/cascade.xml") ){ 
        ROS_ERROR("--(!)Error loading cascade classifier\n");
     }
     image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &signsearcher::imageCb, this);
     image_pub_ = it_.advertise("/image_detections", 1);
     pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &signsearcher::depth_cb, this);
     pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("detection_cloud", 1);
     tilt_sub_ = nh_.subscribe("/cur_tilt_angle", 1, &signsearcher::tilt_angle_cb, this);
     loc_sub_ = nh_.subscribe("/robotpose", 1, &signsearcher::robot_localization_cb, this);
     goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal_pose", 1);

     // Services
     csigns_array = nh_.serviceClient<find_emergency_exit::signs_array>("detect_sign");
     cmotion_goal = nh_.serviceClient<find_emergency_exit::motion_goal>("motion_goal");
     cstop_planner = nh_.serviceClient<find_emergency_exit::on_off>("stop_planner");
     cstop_robot = nh_.serviceClient<find_emergency_exit::on_off>("stop_robot");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) { //(~30Hz)
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame_gray;
    cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
    sign_cascade.detectMultiScale( frame_gray, signs, 1.1, 4, 0, cv::Size(5,5));

    std::vector<double> point(2);
    std::vector<std::vector<double> > frame;
    for( size_t i = 0; i < signs.size(); i++ ){
        point[0] = signs[i].x + signs[i].width/2.0;
        point[1] = signs[i].y + signs[i].height/2.0;
        frame.push_back(point);
        //cv::rectangle(cv_ptr->image, signs[i].tl(), signs[i].br(), cv::Scalar(255,0,0),2,3,0);
    }
    // Add the new points and delete the first points in the bag
    if (bag.size() == Nframes) bag.erase(bag.begin());
    bag.push_back(frame);
    rely_detections = clustering();

    /*for( size_t i = 0; i < bag.size(); i++ ) {
       for( size_t j = 0; j < bag[i].size(); j++ ) {
          cv::circle( cv_ptr->image, cv::Point( bag[i][j][0], bag[i][j][1] ), sqrt(r_pixelmax2), cv::Scalar(0,0,255), 1, 8 );
       }
    }*/

    for( size_t i = 0; i < rely_detections.size(); i++ ) {
       cv::circle( cv_ptr->image, cv::Point( rely_detections[i][0], rely_detections[i][1] ), 5, cv::Scalar(255,255,255), -1, 8 );
    }
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  // Clustering to filter the false positives detections
  std::vector<std::vector<double> > clustering() {
    // Assumption 1: The signs (objects) detected in one frame are further than sqrt(r_pixelmax2)
    // Assumption 2: The detected signs can be clustered with only one cluster
    std::vector<std::vector<double> > reliable_detections;
    if (bag.size() == 0) { return reliable_detections;}

    // Initial clusters
    std::vector<std::vector<std::vector<double> > > clusters;

    int t0=0;
    while (bag[t0].size() == 0) { 
       t0++;
       if (t0 > bag.size()-1) { return reliable_detections;}
    }  
    for (int i=0; i<bag[t0].size(); i++) {  
       std::vector<std::vector<double> > tmp;
       tmp.push_back(bag[t0][i]);
       clusters.push_back(tmp);
    }

    // Clustering by tracking the last element
    for (int t=t0+1; t<bag.size(); t++) {
       for (int ib=0; ib<bag[t].size(); ib++) {
          bool flag = false;
          for (int ic=0; ic<clusters.size(); ic++) {    
             int Nend = clusters[ic].size() - 1;
             double dx = bag[t][ib][0] - clusters[ic][Nend][0];  
             double dy = bag[t][ib][1] - clusters[ic][Nend][1];
             if (dx*dx+dy*dy < r_pixelmax2) {
                clusters[ic].push_back(bag[t][ib]);    //Maybe I must delay the addition??
                flag = true;
                break;
             }
          } 
          // Add new cluster 
          if (flag == false) {
             std::vector<std::vector<double> > tmp;
             tmp.push_back(bag[t][ib]);
             clusters.push_back(tmp);
          } 
       }  
    }
   
    // Select the most reliable detections
    for (int i=0; i<clusters.size(); i++) {  
       if (clusters[i].size() >= pframes*Nframes) {
          int Nend = clusters[i].size() - 1;
          reliable_detections.push_back(clusters[i][Nend]);
       }
    } 
    return reliable_detections;   
  }

  void tilt_angle_cb(const std_msgs::Float64ConstPtr& tilt) {  //(~513Hz)
     th_tilt = tilt->data*M_PI/180.0;
  }
 
  void robot_localization_cb(const geometry_msgs::PoseStamped::ConstPtr& robot) { //(~14Hz)
     tf::Pose pose;
     tf::poseMsgToTF(robot->pose, pose);
     thRobot = tf::getYaw(pose.getRotation());
     xRobot = robot->pose.position.x;
     yRobot = robot->pose.position.y;
     set_goal();
  }

  void depth_cb(const sensor_msgs::PointCloud2ConstPtr& input) {  //(~30Hz)
     pcl::PointCloud<pcl::PointXYZ>::Ptr detectioncloud (new pcl::PointCloud<pcl::PointXYZ>);
     if (abs(th_tilt) < 0.55) {  // 0.55=31° To avoid errors when the tilt angle is moving
        for (size_t is = 0; is < rely_detections.size(); is++) { 
           // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
           pcl::PointCloud<pcl::PointXYZRGB> depth;
           pcl::fromROSMsg (*input, depth);
        
           // Form the cloud for window detection
           int i = rely_detections[is][0], j = rely_detections[is][1];
           pcl::PointXYZ p1, p2, p3, p4, p5;
           //frame_camera_kinect
           p1.x = depth.points[j*640 + i].x;
           p1.y = depth.points[j*640 + i].y;
           p1.z = depth.points[j*640 + i].z;
           if (isnan(p1.x) || isnan(p1.y) || isnan(p1.z)) {  // WATCH OUT: The depth is NAN at the extrems of the image. 
              continue;
           }
           double distance2 = p1.x*p1.x + p1.y*p1.y + p1.z*p1.z;
           if (distance2 > 6.0*6.0) {  // WATCH OUT: To guarantee the sign is near enough to be reliably detected by the detection service
              continue;
           }
           //frame_base_kinect (base_link)
           p2.x = p1.x;
           p2.y = p1.y*cos(th_tilt) - p1.z*sin(th_tilt);
           p2.z = p1.y*sin(th_tilt) + p1.z*cos(th_tilt);
           //frame_map
           p3.x = p2.z;
           p3.y = -p2.x;
           p3.z = 0.0;
           p4.x = p3.x*cos(thRobot) - p3.y*sin(thRobot) + xRobot;
           p4.y = p3.x*sin(thRobot) + p3.y*cos(thRobot) + yRobot;
           p4.z = 0.0;

           // Cluster the new landmark
           std::vector<double> landmark(2);
           landmark[0] = p4.x;
           landmark[1] = p4.y;
           landmark_clustering(landmark);
 
           for (int i=0; i<landmarks_bag.size(); i++) {
              for (int j=0; j<landmarks_bag[i].size(); j++) {
                 p5.x = landmarks_bag[i][j][0];
                 p5.y = landmarks_bag[i][j][1];
                 p5.z = 0.0;
                 detectioncloud->points.push_back(p5);
              }
           }
        }
     }
     // Publish results
     sensor_msgs::PointCloud2 windowcloud_ROS;
     pcl::toROSMsg(*detectioncloud,windowcloud_ROS);
     windowcloud_ROS.header.frame_id = "map"; //input->header.frame_id
     pcl_pub_.publish(windowcloud_ROS);
  }
  
  void landmark_clustering(std::vector<double> landmark) {
     // landmark = (x,y), ld[i] = (xi,yi)
     double dx, dy;
     bool flag = true;
     for (int i=0; i<ld.size(); i++) {
        dx = ld[i][0]-landmark[0];  
        dy = ld[i][1]-landmark[1];    
        if (dx*dx+dy*dy < drmaxc2) {
           // Add the landmark and delete the first landmark in the bag
           // To avoid memory goes to infinite and reduce execution time
           if (landmarks_bag[i].size() == Nlandmarks) landmarks_bag[i].erase(landmarks_bag[i].begin());  
           landmarks_bag[i].push_back(landmark);

           for (int n=0; n<2; n++) { 
              double sum=0.0;
              for (int j=0; j<landmarks_bag[i].size(); j++) {
                 sum = sum + landmarks_bag[i][j][n];
              }
              ld[i][n] = sum/landmarks_bag[i].size();
           }
           flag = false;
           break;
        }
     }
     // Create a new cluster if needed => Detection service
     if (flag) {
        ROS_ERROR("There is a new sign to detect, stop!!!");
        // Stop robot
        find_emergency_exit::on_off srvSR;
        srvSR.request.switch1 = true;
        if (!cstop_robot.call(srvSR)) ROS_ERROR("Failed to call service stop_robot"); 
        if (srvSR.response.isDone) ROS_INFO("The robot was stopped"); 
        else ROS_ERROR("The robot hasn't stopped");
        // Stop planner
        find_emergency_exit::on_off srvSP;
        srvSP.request.switch1 = true;
        if (!cstop_planner.call(srvSP)) ROS_ERROR("Failed to call service stop_planner"); 
        if (srvSP.response.isDone) ROS_INFO("The planner was stopped"); 
        else ROS_ERROR("The planner hasn't stopped");
      
        update_landmarks(); //Call the Detection Service 
        set_goal(); // Select and publish the goal

        // Restart planner 
        srvSP.request.switch1 = false;
        if (!cstop_planner.call(srvSP)) ROS_ERROR("Failed to call service stop_planner"); 
        if (srvSP.response.isDone) ROS_INFO("The planner was on"); 
        else ROS_ERROR("The planner hasn't been on");
        // Restart waypoints_follower
        srvSR.request.switch1 = false;
        if (!cstop_robot.call(srvSR)) ROS_ERROR("Failed to call service stop_robot"); 
        if (srvSR.response.isDone) ROS_INFO("The robot restart to move"); 
        else ROS_ERROR("The robot hasn't restarted to move");
     }
  }

  void set_goal() { 
     double xG = NAN, yG = NAN, thG = NAN;
     // Set the nearest sign as the goal
     double mindist2 = 1.0/0.0; 
     for (int i=0; i<ld.size(); i++) {
        double dx = ld[i][0]-xRobot;  
        double dy = ld[i][1]-yRobot; 
        double dist = dx*dx+dy*dy;
        if (dist < mindist2) {
           mindist2 = dist;
           xG = ld[i][0];
           yG = ld[i][1];
           thG = arrow_directions[i]; 
        }
     }
     publish_goal(xG,yG,thG);
  }

  void update_landmarks() {
     find_emergency_exit::signs_array srvDS;
     if (!csigns_array.call(srvDS)) { ROS_ERROR("Failed to call service detect_sign");}
     
     for (int i=0; i<int(srvDS.response.signs.data.size()/3.0); i++) {
        double x = srvDS.response.signs.data[0 + 3*i];
        double y = srvDS.response.signs.data[1 + 3*i];
        double th = srvDS.response.signs.data[2 + 3*i];

        // Check whether it is a old landmark? If so, just add the arrow direction
        bool flag = true;
        for (int j=0; j<ld.size(); j++) {
           double dx = ld[j][0]-x;  
           double dy = ld[j][1]-y; 
           if (dx*dx+dy*dy < drmaxc2) {
              //arrow_directions[j] = th;
              flag = false;
              break;
           }
        } 
        
        // If it is a new landmark, then updates the landmarks
        if (flag) {
           std::vector<std::vector<double> > vtmp;
           std::vector<double> new_landmark(2);
           new_landmark[0] = x;
           new_landmark[1] = y;
           vtmp.push_back(new_landmark);
           landmarks_bag.push_back(vtmp);
           ld.push_back(new_landmark);
           arrow_directions.push_back(th);
        }
     } 
  }

  void publish_goal(double xG, double yG, double thG) {
     geometry_msgs::PoseStamped goal;

     goal.header.stamp = ros::Time::now();
     goal.header.frame_id = "map";
     goal.pose.position.x = xG; 
     goal.pose.position.y = yG;
     goal.pose.position.z = 0;      
     goal.pose.orientation.x = 0.0;
     goal.pose.orientation.y = 0.0;
     goal.pose.orientation.z = sin(thG/2);
     goal.pose.orientation.w = cos(thG/2);
     goal_pub_.publish(goal);

     find_emergency_exit::motion_goal srvMG;
     srvMG.request.x = xG;
     srvMG.request.y = yG;
     srvMG.request.th = thG;
     if (!cmotion_goal.call(srvMG)) ROS_ERROR("Failed to call service motion_goal"); 
     if (srvMG.response.isDone) ROS_INFO("The goal was set"); 
     else ROS_ERROR("The goal clould not set");
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "signsearcher");
  signsearcher ic;
  ros::spin();
  return 0;
}
