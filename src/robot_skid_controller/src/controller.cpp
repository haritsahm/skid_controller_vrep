#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


#define PI 3.1415926535
#define SUMMIT_XL_MIN_COMMAND_REC_FREQ   5.0
#define SUMMIT_XL_MAX_COMMAND_REC_FREQ   150.0

#define SKID_STEERING                1
#define MECANUM_STEERING             2

#define SUMMIT_XL_WHEEL_DIAMETER	0.25      // Default wheel diameter
#define SUMMIT_XL_D_TRACKS_M    	1.0       // default equivalent W distance (difference is due to slippage of skid steering)
#define SUMMIT_XL_WHEELBASE         0.446     // default real L distance forward to rear axis
#define SUMMIT_XL_TRACKWIDTH        0.408     // default real W distance from left wheels to right wheels


class SkidController
{
public:
  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  // Robot model
  std::string robot_model_;

  // Topics - skid - velocity
  std::string frw_vel_topic_;
  std::string flw_vel_topic_;
  std::string brw_vel_topic_;
  std::string blw_vel_topic_;

  // Joint names - skid - velocity
  std::string joint_front_right_wheel;
  std::string joint_front_left_wheel;
  std::string joint_back_left_wheel;
  std::string joint_back_right_wheel;

  // Joint names - ptz - position
  std::string joint_camera_pan;
  std::string joint_camera_tilt;

  // Topics - ptz
  std::string pan_pos_topic_;
  std::string tilt_pos_topic_;

  // Velocity and position references to low level controllers
  ros::Publisher ref_vel_flw_, ref_vel_frw_, ref_vel_blw_, ref_vel_brw_;
  ros::Publisher ref_pos_pan_, ref_pos_tilt_;


  // Indexes to joint_states
  int frw_vel_, flw_vel_, blw_vel_, brw_vel_;
  int frw_pos_, flw_pos_, blw_pos_, brw_pos_;
  int scissor_pos_;
  int pan_pos_, tilt_pos_;

  // Robot Speeds
  Eigen::Vector3d curr_vel;
  Eigen::Quaterniond curr_orientation;

  // Joint states published by the joint_state_controller of the Controller Manager
  ros::Subscriber joint_state_sub_;

  // Robot Joint States
  sensor_msgs::JointState joint_state_;

  // Command reference
  geometry_msgs::Twist base_vel_msg_;

  // Robot configuration parameters
  double summit_xl_wheel_diameter_;
  double summit_xl_d_tracks_m_;
  double summit_xl_wheelbase_;
  double summit_xl_trackwidth_;

  // Parameter that defines if odom tf is published or not
  bool publish_odom_tf_;

  ros::Subscriber imu_sub_;

  // Publisher for odom topic
  ros::Publisher odom_pub_;

  // Broadcaster for odom tf
  tf::TransformBroadcaster odom_broadcaster;


  SkidController(ros::NodeHandle h) :
      node_handle_(h),  private_node_handle_("~")
  {
      ros::NodeHandle summit_xl_robot_control_node_handle(node_handle_, "robot_skid_controller");

      // Get robot model from the parameters
        if (!private_node_handle_.getParam("model", robot_model_)) {
            ROS_ERROR("Robot model not defined.");
            exit(-1);
            }
        else ROS_INFO("Robot Model : %s", robot_model_.c_str());

      // Skid configuration - topics
      private_node_handle_.param<std::string>("frw_vel_topic", frw_vel_topic_, "/summit_xl/joint_frw_velocity_controller/command");
      private_node_handle_.param<std::string>("flw_vel_topic", flw_vel_topic_, "/summit_xl/joint_flw_velocity_controller/command");
      private_node_handle_.param<std::string>("blw_vel_topic", blw_vel_topic_, "/summit_xl/joint_blw_velocity_controller/command");
      private_node_handle_.param<std::string>("brw_vel_topic", brw_vel_topic_, "/summit_xl/joint_brw_velocity_controller/command");

      // Skid configuration - Joint names
      private_node_handle_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "joint_front_right_wheel");
      private_node_handle_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "joint_front_left_wheel");
      private_node_handle_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "joint_back_left_wheel");
      private_node_handle_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "joint_back_right_wheel");

      // PTZ topics
      private_node_handle_.param<std::string>("joint_camera_pan", joint_camera_pan, "joint_camera_pan");
      private_node_handle_.param<std::string>("joint_camera_tilt", joint_camera_tilt, "joint_camera_tilt");

      // Robot parameters
      if (!private_node_handle_.getParam("summit_xl_wheel_diameter", summit_xl_wheel_diameter_))
        summit_xl_wheel_diameter_ = SUMMIT_XL_WHEEL_DIAMETER;
      if (!private_node_handle_.getParam("summit_xl_d_tracks_m", summit_xl_d_tracks_m_))
        summit_xl_d_tracks_m_ = SUMMIT_XL_D_TRACKS_M;
      ROS_INFO("summit_xl_wheel_diameter_ = %5.2f", summit_xl_wheel_diameter_);
      ROS_INFO("summit_xl_d_tracks_m_ = %5.2f", summit_xl_d_tracks_m_);

      private_node_handle_.param("publish_odom_tf", publish_odom_tf_, true);
      if (publish_odom_tf_) ROS_INFO("PUBLISHING odom->base_footprin tf");
      else ROS_INFO("NOT PUBLISHING odom->base_footprint tf");

      // Subscribe to joint states topic
      joint_state_sub_ = summit_xl_robot_control_node_handle.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &SkidController::jointStateCallback, this);

      // Adevertise reference topics for the controllers
      ref_vel_frw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_frw_velocity_controller/command", 50);
      ref_vel_flw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_flw_velocity_controller/command", 50);
      ref_vel_blw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_blw_velocity_controller/command", 50);
      ref_vel_brw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_brw_velocity_controller/command", 50);

      ref_pos_pan_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_pan_position_controller/command", 50);
      ref_pos_tilt_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl/joint_tilt_position_controller/command", 50);

      odom_pub_ = summit_xl_robot_control_node_handle.advertise<nav_msgs::Odometry>("/summit_xl/odom", 1000);

      // Subscribe to command topic
//      cmd_sub_ = summit_xl_robot_control_node_handle.subscribe<geometry_msgs::Twist>("command", 1, &SummitXLControllerClass::commandCallback, this);

  }
  void initialize()
  {

  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joint_state_ = *msg;
  }



};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_skid_controller");

    ros::NodeHandle n;
//    SummitXLControllerClass sxlrc(n);


    // ros::ServiceServer service = n.advertiseService("set_odometry", &summit_xl_node::set_odometry, &sxlc);
//    sxlrc.spin();

    return (0);


}
