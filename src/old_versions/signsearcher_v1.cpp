/* 
   
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
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "find_emergency_exit/search_sign.h"

int Nframes = 5, Nlandmarks = 20;
double r_pixelmax2 = 30*30, pframes = 1.0, drmaxc2 = 1.0*1.0;

class signsearcher {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_; 
  ros::Subscriber pcl_sub_, odom_sub_,tilt_sub_;
  ros::Publisher pcl_pub_;
  cv::CascadeClassifier sign_cascade;
  std::vector<cv::Rect> signs;
  double th_tilt, xRobot, yRobot, thRobot;
  std::vector<std::vector<std::vector<double> > > bag, landmarks_bag;
  std::vector<std::vector<double> > rely_detections, ld;
  ros::ServiceClient csearch_sign;

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
     odom_sub_ = nh_.subscribe("/odom", 1, &signsearcher::odom_cb, this);
     csearch_sign = nh_.serviceClient<find_emergency_exit::search_sign>("search_sign");
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
 
  void odom_cb(const nav_msgs::Odometry::ConstPtr& currodom) { //(~14Hz)
     tf::Pose pose;
     tf::poseMsgToTF(currodom->pose.pose, pose);
     thRobot = tf::getYaw(pose.getRotation());
     xRobot = currodom->pose.pose.position.x;
     yRobot = currodom->pose.pose.position.y;
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
           //frame_base_kinect
           p2.x = p1.x;
           p2.y = p1.y*cos(th_tilt) - p1.z*sin(th_tilt);
           p2.z = p1.y*sin(th_tilt) + p1.z*cos(th_tilt);
           //frame_odometry
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
     windowcloud_ROS.header.frame_id = "odom"; //input->header.frame_id
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
     // Create a new cluster if needed
     if (flag) {
        std::vector<std::vector<double> > vtmp;
        vtmp.push_back(landmark);
        landmarks_bag.push_back(vtmp);
        ld.push_back(landmark);

        ROS_ERROR("There is a new sign to detect, stop!!!");
        find_emergency_exit::search_sign srvSS;
        if (!csearch_sign.call(srvSS)) { ROS_ERROR("Failed to call service search_sign");}
     }
  }
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "signsearcher");
  signsearcher ic;
  ros::spin();
  return 0;
}
