/* 
   It provides a service for detecting the emergency sign. 
   After Tdec seconds, it chooses the most reliable detection giving sign direction to
   move the robot towards the exit 
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "find_emergency_exit/detect_sign.h"



const double MaxVerticalAng = (43.0/2.0)/180.0*M_PI, H_Tol = 0.2;
const double drmaxc2 = 0.5*0.5;
const double Tdec = 10.0;
const int Nclustersmin = 10;
const int NminPoints = 10;

std::vector<std::vector<std::vector<double> > > clusters;
std::vector<std::vector<double> > dclusters;
int imax; 
std::string Imgfolder="/home/erik/catkin_ws/src/find_emergency_exit/";
/* 0: untold, 1: forward, 2:right, 3:left */
std::string arr[] = { "T2.png","T1.png","T1_up.png","T1_down.png","T3a.png","T3a_up.png","T3a_down.png"}; 
std::vector<std::string> templFiles(arr, arr+7); 
cv::Mat imgCurrent, reliable_det;
double xG, yG, thG;
bool flagOdom;

class SignDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber pcl_sub_, odom_sub_;
  ros::Publisher pcl_pub_, plane_pub_, gcluster_pub_, bestcluster_pub_;
  cv::CascadeClassifier sign_cascade;
  std::vector<cv::Rect> signs, signs2;
  ros::ServiceServer srvdetections;
  
public:
  SignDetector()
    : it_(nh_)
  {
     srvdetections = nh_.advertiseService("detect_sign", &SignDetector::detection_cb, this);
    
     image_pub_ = it_.advertise("/image_detections", 1);
     pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("window_cloud", 1);
     plane_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("plane_pose", 1);
     gcluster_pub_ = nh_.advertise<geometry_msgs::PoseArray>("clusters", 1);
     bestcluster_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("bestcluster", 1);
  }

  bool detection_cb(find_emergency_exit::detect_sign::Request  &req, find_emergency_exit::detect_sign::Response &res) {
     flagOdom = true;
     dclusters.clear();
     clusters.clear();
     res.x = NAN;
     res.y = NAN;
     res.th = NAN;
     
     if( !sign_cascade.load("/home/erik/catkin_ws/src/find_emergency_exit/cascade.xml") ){ 
        ROS_ERROR("--(!)Error loading cascade classifier\n");
        return true;
     }
     // Subscribe to image and point cloud
     image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &SignDetector::imageCb, this);
     pcl_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &SignDetector::depth_cb, this);
    
     ros::Time start_time = ros::Time::now();
     while (ros::ok() && (ros::Time::now() - start_time).toSec() < Tdec) {
        ros::spinOnce();
     }
     image_sub_.shutdown();
     pcl_sub_.shutdown();

     // Recognize the arrow direction
     if (dclusters.size() > 0) {
        if (clusters[imax].size() > Nclustersmin) {
           MatchingMethod(reliable_det);

           // (xs,ys,ths) From kinect frame to odom frame
           odom_sub_ = nh_.subscribe("/odom", 1, &SignDetector::odom_cb, this);
           while (ros::ok() && flagOdom) {
              ros::spinOnce();
           }
           odom_sub_.shutdown();

           res.x = xG;
           res.y = yG;
           res.th = thG;
        }
     }
     
     for(int i=0; i<clusters.size(); i++) {
        std::cout << "cluster " << i << ": " << clusters[i].size() << std::endl;
     }
     return true;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat frame_gray;
    
    cv::cvtColor(cv_ptr->image, frame_gray, cv::COLOR_BGR2GRAY);
    sign_cascade.detectMultiScale( frame_gray, signs, 1.1, 4, 0, cv::Size(5,5));
    for( size_t i = 0; i < signs.size(); i++ ){
        cv::rectangle(cv_ptr->image, signs[i].tl(), signs[i].br(), cv::Scalar(255,0,0),2,3,0);
    }
    cv_ptr->image.copyTo(imgCurrent);
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

  void depth_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
     for (size_t is = 0; is < signs.size(); is++) {
        // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZRGB> depth;
        pcl::fromROSMsg (*input, depth);
        
        // Form the cloud for window detection
        int w = signs[is].width, h = signs[is].height, i0, x = signs[is].x, y = signs[is].y;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr windowcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        windowcloud->width = w;
        windowcloud->height = h;
        for(int j=y; j<y+h; j++){
           i0 = j*640 + x;
           for (int i=i0; i<i0+w; i++){
              windowcloud->points.push_back(depth.points[i]);
           }
        }

        // Compute the coefficients of a plane equation for the detection window
        pcl::ModelCoefficients coefficients;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(windowcloud->makeShared());
        seg.segment(*inliers, coefficients); 

        // Compute the cloud points closer to the plane
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudfilter (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(windowcloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudfilter);

        // Compute the centroid of window cloud
        float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        int count_x = 0, count_y = 0, count_z = 0;
        float xd, yd, zd;
        for(int i=0; i<cloudfilter->points.size(); i++){
           if (!std::isnan(cloudfilter->points[i].x)){ 
              sum_x += cloudfilter->points[i].x;
              count_x += 1;
           }
           if (!std::isnan(cloudfilter->points[i].y)){ 
              sum_y += cloudfilter->points[i].y;
              count_y += 1;
           }
           if (!std::isnan(cloudfilter->points[i].z)){ 
              sum_z += cloudfilter->points[i].z;
              count_z += 1;
           }
        }
        if ((count_x < NminPoints) || (count_y < NminPoints) || (count_z < NminPoints)) {
           std::cout << "There are less than " << NminPoints << " in the detection window" << std::endl;
           continue;
        }
        xd = sum_x / (float) count_x;
        yd = sum_y / (float) count_y;
        zd = sum_z / (float) count_z;

        float a0 = coefficients.values[0];
        float a1 = coefficients.values[1];
        float a2 = coefficients.values[2];
        float a3 = coefficients.values[3];

        //std::cerr << "coefficients: " << a0 << a1 << a2 << a3 << std::endl;
        
        // Compute the pose (position, orientation) of plane
        geometry_msgs::PoseStamped plane;
        plane.header.stamp = ros::Time::now();
        plane.header.frame_id = input->header.frame_id;
        plane.pose.position.x = xd; 
        plane.pose.position.y = yd;
        plane.pose.position.z = zd;
        float theta = acos(a0/sqrt(a0*a0+a1*a1+a2*a2));       
        plane.pose.orientation.x = 0.0;
        plane.pose.orientation.y = -a2/sqrt(a1*a1+a2*a2)*sin(theta/2);
        plane.pose.orientation.z = a1/sqrt(a1*a1+a2*a2)*sin(theta/2);
        plane.pose.orientation.w = cos(theta/2);
        plane_pub_.publish(plane);

        // Clustering
        std::vector<double> signpose(7);
        signpose[0] = xd;
        signpose[1] = yd;
        signpose[2] = zd;
        signpose[3] = a0;
        signpose[4] = a1;
        signpose[5] = a2;
        signpose[6] = a3;
        clustering(signpose);

        // Save the most reliable detection 
        if (dclusters.size() > 0) {
            if (clusters[imax].size() > Nclustersmin) {
                double ddx = dclusters[imax][0]-xd;  
                double ddy = dclusters[imax][1]-yd;   
                double ddz = dclusters[imax][2]-zd;  
                if (ddx*ddx+ddy*ddy+ddz*ddz < drmaxc2) {
                   reliable_det = imgCurrent(signs[is]);
                }
            }
        }

        // Publish results
        sensor_msgs::PointCloud2 windowcloud_ROS;
        pcl::toROSMsg(*cloudfilter,windowcloud_ROS);
        windowcloud_ROS.header.frame_id = input->header.frame_id;
        pcl_pub_.publish(windowcloud_ROS);
        
     }
     
     // Compute the most reliable cluster
     int Nmax = 0;
     for (int i=0; i<clusters.size(); i++) {
        if (clusters[i].size() > Nmax) {
           Nmax = clusters[i].size();
           imax = i;
        }
     }
     if (dclusters.size() > 0) {
        geometry_msgs::PoseStamped bestcluster;
        bestcluster.header.stamp = ros::Time::now();
        bestcluster.header.frame_id = input->header.frame_id;
        bestcluster.pose.position.x = dclusters[imax][0]; 
        bestcluster.pose.position.y = 0.0;    //dclusters[imax][1]
        bestcluster.pose.position.z = dclusters[imax][2];
        float theta = acos(dclusters[imax][3]/sqrt(dclusters[imax][3]*dclusters[imax][3]+dclusters[imax][4]*dclusters[imax][4]+dclusters[imax][5]*dclusters[imax][5]));
        float tmpvar = dclusters[imax][5]*dclusters[imax][5]+dclusters[imax][4]*dclusters[imax][4];
        bestcluster.pose.orientation.x = 0.0;
        bestcluster.pose.orientation.y = -dclusters[imax][5]/sqrt(tmpvar)*sin(theta/2); 
        bestcluster.pose.orientation.z = dclusters[imax][4]/sqrt(tmpvar)*sin(theta/2);
        bestcluster.pose.orientation.w = cos(theta/2);
        bestcluster_pub_.publish(bestcluster);
     }
     
     // Publish the clusters
     geometry_msgs::PoseArray gcluster;
     gcluster.header.stamp = ros::Time::now();
     gcluster.header.frame_id = input->header.frame_id;
     //std::cout << "Clusters" << std::endl;
     for (int i=0; i<dclusters.size(); i++) {
        geometry_msgs::PoseStamped cpose;
        //std::cout << dclusters[i][0] << " " << dclusters[i][1] << " " << dclusters[i][2] << std::endl;
        //std::cout << "a0: " << dclusters[i][3] << "a1: " << dclusters[i][4] << "a2: " << dclusters[i][5] << "a3: " << dclusters[i][6] << std::endl;
        cpose.pose.position.x = dclusters[i][0]; 
        cpose.pose.position.y = dclusters[i][1];
        cpose.pose.position.z = dclusters[i][2];   
        float theta = acos(dclusters[i][3]/sqrt(dclusters[i][3]*dclusters[i][3]+dclusters[i][4]*dclusters[i][4]+dclusters[i][5]*dclusters[i][5]));
        float tmpvar = dclusters[i][5]*dclusters[i][5]+dclusters[i][4]*dclusters[i][4]; 
        cpose.pose.orientation.x = 0.0;
        cpose.pose.orientation.y = -dclusters[i][5]/sqrt(tmpvar)*sin(theta/2);
        cpose.pose.orientation.z = dclusters[i][4]/sqrt(tmpvar)*sin(theta/2);
        cpose.pose.orientation.w = cos(theta/2);
        gcluster.poses.push_back(cpose.pose);
     }
     /*std::cout << " Distribucion " << std::endl;
     for (int i=0; i<clusters.size(); i++) {
        std::cout << "N" << i << " : " << clusters[i].size() << std::endl;
     }*/
     gcluster_pub_.publish(gcluster);
     
  }

  void clustering(std::vector<double> signpose) {
      // To add elements to cluster
      // state = (x,y,z,a0,a1,a2,a3)
      // dclusters[i] = (xi,yi,zi,a0i,a1i,a2i,a3i)
      double dx, dy, dz;
      bool flag = true;
      for (int i=0; i<dclusters.size(); i++) {
         dx = dclusters[i][0]-signpose[0];  
         dy = dclusters[i][1]-signpose[1];   
         dz = dclusters[i][2]-signpose[2];  
         if (dx*dx+dy*dy+dz*dz < drmaxc2) {
            clusters[i].push_back(signpose);
            for (int n=0; n<signpose.size(); n++) { 
               double sum=0.0;
               for (int j=0; j<clusters[i].size(); j++) {
                  sum = sum + clusters[i][j][n];
               }
               dclusters[i][n] = sum/clusters[i].size();
            }
            flag = false;
            break;
         }
      }
      // Create a new cluster if needed
      if (flag) {
         std::vector<std::vector<double> > vtmp;
         vtmp.push_back(signpose);
         clusters.push_back(vtmp);
         dclusters.push_back(signpose);
      }
  }


  void MatchingMethod(cv::Mat img) {
     
     cv::Mat result, templ;
     int result_cols, result_rows, imax2 = -1;
     double minVal, maxVal, bestVal = -100.0; 
     cv::Point minLoc, maxLoc;
     cv::cvtColor(img, img, CV_RGB2GRAY);

     // Increase size if needed 
     if (img.rows < 45) {
        cv::resize(img, img, cv::Size(), 45.0/img.rows, 45.0/img.rows, cv::INTER_LINEAR);
     }

     // Nearest Neighbor comparison 
     for (int i=0; i<templFiles.size(); i++) { 
        templ = cv::imread(Imgfolder + templFiles[i]);
        cv::cvtColor(templ, templ, CV_RGB2GRAY);
        result.create( img.cols - templ.cols + 1, img.rows - templ.rows + 1, CV_32FC1 );
        cv::matchTemplate( img, templ, result, CV_TM_CCOEFF_NORMED );
        cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
        std::cout << "Maximum Correlation: " << maxVal << std::endl;
        if (maxVal > bestVal) {
           bestVal = maxVal;
           imax2 = i;
        }
     }
     xG = dclusters[imax][2];   //Convert to robot's frame
     yG = -dclusters[imax][0];  //Convert to robot's frame
     double ax = dclusters[imax][3], az = dclusters[imax][5];
     if (az < 0) { az = -az; ax = -ax; }
     double thforward = atan2(az,ax) - M_PI/2;//Convert to robot's frame
     if (imax2 == 0) {
        thG = thforward;      
        std::cout << "Forward direction" << std::endl;
     } else if (imax2 >= 1 && imax2 <= 3){
        thG = thforward - M_PI/2;    
        std::cout << "Right direction" << std::endl;
     } else {
        thG = thforward + M_PI/2;     
        std::cout << "Left direction" << std::endl;
     }
     return;
  }

  void odom_cb(const nav_msgs::Odometry::ConstPtr& currodom) { //(~14Hz)
     //(xs,ys,ths) sign's pose respect to odometry frame
     flagOdom = false;
     tf::Pose pose;
     tf::poseMsgToTF(currodom->pose.pose, pose);
     double thR = tf::getYaw(pose.getRotation());
     double xR = currodom->pose.pose.position.x;
     double yR = currodom->pose.pose.position.y;
     double xtmp = xG*cos(thR) - yG*sin(thR) + xR;
     double ytmp = xG*sin(thR) + yG*cos(thR) + yR;
     xG = xtmp;
     yG = ytmp;
     thG = thR + thG;
     
     return;
  }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "SignDetector");
  SignDetector ic;
  ros::spin();
  return 0;
}
