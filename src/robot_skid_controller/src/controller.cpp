#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <ros/ros.h>
#include <ros/param.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;

#define SGN(x) (x > 0) ? 1 : ((x < 0) ? -1 : 0)
#define PI 3.1415926535
#define SUMMIT_XL_MIN_COMMAND_REC_FREQ   5.0
#define SUMMIT_XL_MAX_COMMAND_REC_FREQ   150.0

#define SKID_STEERING                1
#define MECANUM_STEERING             2

#define SUMMIT_XL_WHEEL_DIAMETER	0.25      // Default wheel diameter
#define SUMMIT_XL_D_TRACKS_M    	1.0       // default equivalent W distance (difference is due to slippage of skid steering)
#define SUMMIT_XL_WHEELBASE         0.446     // default real L distance forward to rear axis
#define SUMMIT_XL_TRACKWIDTH        0.408     // default real W distance from left wheels to right wheels

#define  XICR 0.02

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
  int pan_pos_, tilt_pos_;

  // Robot Speeds
  Eigen::Vector3d curr_vel;
  Eigen::Quaterniond curr_orientation;

  // Joint states published by the joint_state_controller of the Controller Manager
  ros::Subscriber joint_state_sub_;

  // Robot Joint States
  sensor_msgs::JointState joint_state_;
  geometry_msgs::PoseStamped robot_pose_;
  geometry_msgs::Point robot_pos;

  // Command reference
  geometry_msgs::Twist base_vel_msg_;

  // Robot configuration parameters
  double summit_xl_wheel_diameter_;
  double summit_xl_d_tracks_m_;
  double summit_xl_wheelbase_;
  double summit_xl_trackwidth_;

  // Parameter that defines if odom tf is published or not
  bool publish_odom_tf_;

  // Publisher for odom topic
  ros::Publisher odom_pub_;

  // Broadcaster for odom tf
  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener listener;

  // path trajectory point
  std::vector<Eigen::Affine3d> traj_point;
  Eigen::Vector3d vel_law;
  int num_points, current_index;
  bool finished;
  double rho, gamma;
  double k_1, k_2;
  double C, v_max;
  bool stop;

  SkidController(ros::NodeHandle h) :
    node_handle_(h),  private_node_handle_("~")
  {
    ros::NodeHandle summit_xl_robot_control_node_handle(node_handle_, "robot_skid_controller");

    string config_path = "";
    node_handle_.param<std::string>("config_file_path", config_path, "");
    loadConfig(config_path);
    std::string bezier_path = ros::package::getPath("robot_skid_controller") + "/path_bezier_points.csv";
    std::cout << "bezier Path " << bezier_path << std::endl;
    loadPath(bezier_path);
    // Skid configuration - topics
    private_node_handle_.param<std::string>("frw_vel_topic", frw_vel_topic_, "/summit_xl_robot_control/joint_frw_velocity_controller/command");
    private_node_handle_.param<std::string>("flw_vel_topic", flw_vel_topic_, "/summit_xl_robot_control/joint_flw_velocity_controller/command");
    private_node_handle_.param<std::string>("blw_vel_topic", blw_vel_topic_, "/summit_xl_robot_control/joint_blw_velocity_controller/command");
    private_node_handle_.param<std::string>("brw_vel_topic", brw_vel_topic_, "/summit_xl_robot_control/joint_brw_velocity_controller/command");

    // Skid configuration - Joint names
    private_node_handle_.param<std::string>("joint_front_right_wheel", joint_front_right_wheel, "joint_front_right_wheel");
    private_node_handle_.param<std::string>("joint_front_left_wheel", joint_front_left_wheel, "joint_front_left_wheel");
    private_node_handle_.param<std::string>("joint_back_left_wheel", joint_back_left_wheel, "joint_back_left_wheel");
    private_node_handle_.param<std::string>("joint_back_right_wheel", joint_back_right_wheel, "joint_back_right_wheel");

    // PTZ topics
    private_node_handle_.param<std::string>("joint_camera_pan", joint_camera_pan, "joint_camera_pan");
    private_node_handle_.param<std::string>("joint_camera_tilt", joint_camera_tilt, "joint_camera_tilt");

    //    private_node_handle_.param("publish_odom_tf", publish_odom_tf_, false);
    //    if (publish_odom_tf_) ROS_INFO("PUBLISHING odom->base_footprin tf");
    //    else ROS_INFO("NOT PUBLISHING odom->base_footprint tf");

    // Subscribe to joint states topic
    joint_state_sub_ = summit_xl_robot_control_node_handle.subscribe<sensor_msgs::JointState>("/summit_xl/joint_states", 1, &SkidController::jointStateCallback, this);

    // Adevertise reference topics for the controllers
    ref_vel_frw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_frw_velocity_controller/command", 50);
    ref_vel_flw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_flw_velocity_controller/command", 50);
    ref_vel_blw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_blw_velocity_controller/command", 50);
    ref_vel_brw_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_brw_velocity_controller/command", 50);

    ref_pos_pan_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_pan_position_controller/command", 50);
    ref_pos_tilt_ = summit_xl_robot_control_node_handle.advertise<std_msgs::Float64>( "/summit_xl_robot_control/joint_tilt_position_controller/command", 50);

    odom_pub_ = summit_xl_robot_control_node_handle.advertise<nav_msgs::Odometry>("/summit_xl/odom", 1000);

    // Subscribe to command topic
    //      cmd_sub_ = summit_xl_robot_control_node_handle.subscribe<geometry_msgs::Twist>("command", 1, &SummitXLControllerClass::commandCallback, this);
    initialize();
  }

  void loadConfig(const string path)
  {
    std::cout << "Found Path : " << path.c_str() << std::endl;
    YAML::Node config = YAML::LoadFile(path.c_str());
    YAML::Node robot_config = config["robot_config"];
    YAML::Node robot_control = config["robot_control"];


    summit_xl_wheel_diameter_ = robot_config["wheel_diameter"].as<double>();
    summit_xl_d_tracks_m_ =  robot_config["track_dist"].as<double>();

    C = robot_control["C"].as<double>();
    k_1 = robot_control["k_1"].as<double>();
    k_2 = robot_control["k_2"].as<double>();
    v_max = robot_control["v_max"].as<double>();

  }

  int loadPath(const string path)
  {
    // File pointer
    fstream fin(path.c_str());

    vector<string> row;
    string word, temp;

    while (fin >> temp) {

      row.clear();
      stringstream s(temp);
      while (getline(s, word, ',')) {

        // add all the column data
        // of a row to a vector
        row.push_back(word);
      }

      Eigen::AngleAxisd rot = Eigen::AngleAxisd(stoi(row[4])*M_PI/180, Eigen::Vector3d::UnitZ());
      Eigen::Vector3d pos = Eigen::Vector3d(stoi(row[0]), stoi(row[1]), stoi(row[2]));

      Eigen::Affine3d point_pose;
      point_pose.translation() = pos;
      point_pose.linear() = rot.matrix();

      traj_point.push_back(point_pose);
    }

    std::cout << "Total Point : " << traj_point.size() << std::endl;
  }

  void initialize()
  {
    ROS_INFO("SummitXLControllerClass::starting");

    std::vector<string> joint_names = joint_state_.name;
    frw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_right_wheel)) - joint_names.begin();
    flw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_front_left_wheel)) - joint_names.begin();
    blw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_left_wheel)) - joint_names.begin();
    brw_vel_ = find (joint_names.begin(),joint_names.end(), string(joint_back_right_wheel)) - joint_names.begin();

    // For publishing the ptz joint state
    pan_pos_ = find(joint_names.begin(), joint_names.end(), string(joint_camera_pan)) - joint_names.begin();
    tilt_pos_ = find(joint_names.begin(), joint_names.end(), string(joint_camera_tilt)) - joint_names.begin();

    num_points = traj_point.size();
    current_index = 10;
    finished = true;
    rho = gamma = 0;
  }

  void jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joint_state_ = *msg;
  }

  void robotPoscallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    robot_pose_ = *msg;
    robot_pos = robot_pose_.pose.position;
  }

  void updateControl(tf::Vector3 transf_pos, tf::Quaternion transf_rot, Eigen::Vector3d err_ref, double err_traj_head)
  {
    double robot_heading = tf::getYaw(transf_rot);
    rho = SGN(transf_pos.x())*sqrt(pow(transf_pos.x(),2)+pow(transf_pos.y(),2));
    gamma = atan((SGN(transf_pos.x())*transf_pos.y())/abs(transf_pos.x()))-robot_heading;

    vel_law.x() =rho*(k_2*(gamma+robot_heading)*sin(gamma)-k_1*cos(gamma));
    vel_law.z() = (rho/XICR)*(k_2*(gamma+robot_heading)*cos(gamma)+k_1*sin(gamma));


    //adaptif
    double Vl = vel_law.x() + -C*vel_law.z();
    double Vr = vel_law.x() + C*vel_law.z();

    double vreq = max(abs(Vl), abs(Vr));
    double corr_factor = v_max/vreq;

    if(corr_factor<1)
    {
      vel_law.x() =rho*((k_2*corr_factor)*(gamma+robot_heading)*sin(gamma)-(k_1*corr_factor)*cos(gamma));
      vel_law.z() = (rho/XICR)*((k_2*corr_factor)*(gamma+robot_heading)*cos(gamma)+(k_1*corr_factor)*sin(gamma));
    }
  }

  Affine3d getNextPoint()
  {
    return traj_point[current_index];
  }

  void publishSpeed()
  {
    // Axis are not reversed in the omni (swerve) configuration
    std_msgs::Float64 fr_ref_msg;
    std_msgs::Float64 fl_ref_msg;

    if(!stop)
    {
      fl_ref_msg.data = (vel_law.x() + -C*vel_law.z())/(summit_xl_wheel_diameter_*0.5);
      fr_ref_msg.data = (vel_law.x() + C*vel_law.z())/(summit_xl_wheel_diameter_*0.5);
    }

    else
    {
      fr_ref_msg.data = 0;
      fl_ref_msg.data = 0;
    }

    ref_vel_frw_.publish( fr_ref_msg );
    ref_vel_flw_.publish( fl_ref_msg );
    ref_vel_blw_.publish( fl_ref_msg );
    ref_vel_brw_.publish( fr_ref_msg );
  }

  void mainLoop()
  {
    ros::Rate r(50);  // 50.0

    while(ros::ok())
    {

      tf::StampedTransform transform;
      try{
        listener.lookupTransform("world", "robot_base",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      Affine3d ref_point;

      if(finished)
      {
        ref_point = getNextPoint();
        finished=false;
      }

            Eigen::Affine3d robot_pos;

            tf::Quaternion transf_rot; transf_rot = transform.getRotation();
            Eigen::Quaterniond robot_rot(transf_rot.w(), transf_rot.x(), transf_rot.y(), transf_rot.z());

            tf::Vector3 transf_pos = transform.getOrigin();
            robot_pos.translation() = Eigen::Vector3d(transf_pos.x(), transf_pos.y(), 0);
            robot_pos.linear() = robot_rot.toRotationMatrix();

            Vector3d point_ref_pos = ref_point.translation();

            Eigen::Vector3d e_diff = ref_point.inverse()*robot_pos.translation();
            double err_traj = atan2(e_diff.y(), e_diff.x());

            Eigen::Vector3d error_traj(e_diff.x(), e_diff.y(), err_traj);

            updateControl(transf_pos, transf_rot, e_diff, err_traj);

            std::cout << "getting to target : \n" << ref_point.matrix() << std::endl;
            std::cout << "from position : \n" << robot_pos.matrix() << std::endl;

            std::cout << "Error : " << error_traj.norm() << std::endl;
            publishSpeed();


            //update next target
            if(error_traj.norm() < 0.1)
            {
              if(current_index < traj_point.size())
                current_index++;
              else
              {
                current_index = 0;
                stop=true;
              }

              std::cout << "Update Index : " << current_index << std::endl;

              finished=true;
            }

      ros::spinOnce();
      r.sleep();
    }
  }



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_skid_controller");

  ros::NodeHandle n;
  SkidController controller(n);


  // ros::ServiceServer service = n.advertiseService("set_odometry", &summit_xl_node::set_odometry, &sxlc);
  controller.mainLoop();

  return (0);


}
