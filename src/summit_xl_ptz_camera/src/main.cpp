#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <opencv2/opencv.hpp>

class ImageHandler
{
public:
  ImageHandler(const ros::NodeHandle &nh)
    : nh_(nh),
      it_(nh_)
  {
    ptz_cam = it_.subscribe("summit_xl_robot/ptz_camera_raw", 10, &ImageHandler::image_subscriber, this);
    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &ImageHandler::jointStateCallback, this);


    std::vector<std::string> joint_names = joint_state_.name;
    // For publishing the ptz joint state
    pan_pos_ = std::find(joint_names.begin(), joint_names.end(), std::string("joint_camera_pan")) - joint_names.begin();
    tilt_pos_ = std::find(joint_names.begin(), joint_names.end(), std::string("joint_camera_tilt")) - joint_names.begin();

  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joint_state_ = *msg;
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



  }

  void loop()
  {
    while(ros::ok())
    {

    }

    ros::shutdown();
  }

private:
  ros::NodeHandle nh_;

  image_transport::Subscriber ptz_cam;
  ros::Subscriber joint_state_sub_;
  sensor_msgs::JointState joint_state_;

  image_transport::ImageTransport it_;
  bool image_received;

  int pan_pos_, tilt_pos_;


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sumit_xl_ptz_camera");
  ros::NodeHandle nh;
  ImageHandler image_handler(nh);

  //  ros::spin();

  ROS_INFO("Hello world!");
}
