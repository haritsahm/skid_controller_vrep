#include <ros/ros.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <cmath>

using namespace cv;

class ImageHandler
{
public:
  ImageHandler(const ros::NodeHandle &nh)
    : nh_(nh),
      it_(nh_),
      FOV_WIDTH(32.0 * M_PI / 180),
      FOV_HEIGHT(32.0 * M_PI / 180)
  {
    ptz_cam = it_.subscribe("summit_xl_robot/ptz_camera_raw", 10, &ImageHandler::image_subscriber, this);
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &ImageHandler::jointStateCallback, this);
    laser_scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/summit_xl_robot/laser_scan", 1, &ImageHandler::laserScanCallback, this);
    req_stop = nh_.advertise<std_msgs::String>("summit_xl_robot/req_stop", 10);


    std::vector<std::string> joint_names = joint_state_.name;
    // For publishing the ptz joint state
    pan_pos_ = std::find(joint_names.begin(), joint_names.end(), std::string("joint_ptz_pan")) - joint_names.begin();
    tilt_pos_ = std::find(joint_names.begin(), joint_names.end(), std::string("joint_ptz_tilt")) - joint_names.begin();

    ref_pos_pan_ = nh_.advertise<std_msgs::Float64>( "/summit_xl_robot_control/ptz_pan_joint/command", 50);
    ref_pos_tilt_ = nh_.advertise<std_msgs::Float64>( "/summit_xl_robot_control/ptz_tilt_joint/command", 50);

    low_H = 31; low_S = 146; low_V = 0;
    high_H = 129; high_S = 215; high_V=255;
    prev_pan_val = prev_tilt_val = tilt_val = 0;
    pan_val = 90;
    image_received = false;


    namedWindow(window_detection_name);
    // Trackbars to set thresholds for HSV values
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H);
    createTrackbar("Low S", window_detection_name, &low_S, max_value);
    createTrackbar("High S", window_detection_name, &high_S, max_value);
    createTrackbar("Low V", window_detection_name, &low_V, max_value);
    createTrackbar("High V", window_detection_name, &high_V, max_value);
    createTrackbar("Pan", window_detection_name, &pan_val, 180);
    createTrackbar("Tilt", window_detection_name, &tilt_val, 60);
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joint_state_ = *msg;
  }

  void laserScanCallback(const sensor_msgs::LaserScanConstPtr &msg)
  {
    laser_data = *msg;
  }

  void image_subscriber(const sensor_msgs::ImageConstPtr& msg)
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
    image_in = cv_ptr->image;
    image_received = true;
  }

  void processing()
  {
    Mat thres_image, frame_HSV;
    cvtColor(image_in, frame_HSV, COLOR_BGR2HSV);
    inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), thres_image);

    // Floodfill from point (0, 0)
    Mat im_floodfill = thres_image.clone();
    floodFill(im_floodfill, cv::Point(0,0), Scalar(255));

    // Invert floodfilled image
    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;

    findContours( im_floodfill_inv, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Get the moments
    std::vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mu[i] = moments( contours[i], false ); }

    ///  Get the mass centers:
    std::vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
       { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

    /// Draw contours
    Mat drawing = Mat::zeros( im_floodfill_inv.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
       {
         Scalar color = Scalar( 242, 12, 33 );
         drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
         circle( drawing, mc[i], 4, color, -1, 8, 0 );
       }

    float joint_pan_ =  joint_state_.position[4];
    float joint_tilt_ =  joint_state_.position[5];

    std::vector<Point2f> point_angle(mc.size());
    for(int idx=0; idx< mc.size();idx++)
    {
      Point2f mc_norm;
      mc_norm.x = mc[idx].x / image_in.cols * 2 - 1;    // x (-1 ~ 1)
      mc_norm.y = mc[idx].y / image_in.rows * 2 - 1;    // y (-1 ~ 1)

      point_angle[idx].x = -atan(mc_norm.x * tan(FOV_WIDTH));
      point_angle[idx].y = -atan(mc_norm.y * tan(FOV_HEIGHT));

      if(std::isnan(point_angle[idx].x) || std::isnan(point_angle[idx].y)) continue;

      double err_angle = point_angle[idx].x + joint_pan_;

      double scan_range = laser_data.angle_max-laser_data.angle_min;
      int data_index = int(floor((err_angle-(laser_data.angle_min))/laser_data.angle_increment));

      double dist = laser_data.ranges[data_index];
//      std::cout << "err_angle : " << err_angle << std::endl;
      std::cout << err_angle << " || " << " index " << data_index << " :" << dist << std::endl;
    }



    cv::imshow("Flood Fill", drawing);
    cv::imshow(window_detection_name, thres_image);
    cv::waitKey(2);
  }

  void ptz_joint()
  {
    std_msgs::Float64 pan_, tilt_;
    if(pan_val != prev_pan_val)
    {
      pan_.data = -(pan_val-90);
      ref_pos_pan_.publish(pan_);
      prev_pan_val = pan_val;
    }

    if(tilt_val != prev_tilt_val)
    {
      tilt_.data = tilt_val;
      ref_pos_tilt_.publish(tilt_);
      prev_tilt_val = tilt_val;
    }

  }
  void loop()
  {
    ros::Rate rate(30);
    while(ros::ok())
    {
      if(image_received)
        processing();
      ptz_joint();

      image_received = false;

      ros::spinOnce();
      rate.sleep();
    }
    cv::destroyAllWindows();
    ros::shutdown();
  }

private:
  ros::NodeHandle nh_;

  image_transport::Subscriber ptz_cam;
  ros::Subscriber joint_state_sub_, laser_scan_sub_;
  ros::Publisher req_stop;
  ros::Publisher ref_pos_pan_, ref_pos_tilt_;
  sensor_msgs::JointState joint_state_;
  sensor_msgs::LaserScan laser_data;

  image_transport::ImageTransport it_;
  bool image_received;

  int pan_pos_, tilt_pos_;

  cv::Mat image_in, image_out;

  const int max_value_H = 360/2;
  const int max_value = 255;
  const cv::String window_detection_name = "Object Detection";
  int low_H, low_S, low_V;
  int high_H, high_S, high_V;
  int pan_val, tilt_val, prev_pan_val, prev_tilt_val;

  double FOV_WIDTH, FOV_HEIGHT;




};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sumit_xl_ptz_camera");
  ros::NodeHandle nh;
  ImageHandler image_handler(nh);
  image_handler.loop();
  return 0;
}
