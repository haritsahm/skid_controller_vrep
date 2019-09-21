#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
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
#include <visualization_msgs/Marker.h>

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
  ros::Publisher marker_pub;
  visualization_msgs::Marker points, line_strip, line_list;

  // path trajectory point
  std::vector<Eigen::Vector3d> traj_point;
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

    marker_pub = summit_xl_robot_control_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    odom_pub_ = summit_xl_robot_control_node_handle.advertise<nav_msgs::Odometry>("/summit_xl/odom", 1000);

    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "world";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.1;
    points.scale.y = 0.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

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

  void loadPath(const string path)
  {
    // File pointer
    std::cout << "load Path" << path<< std::endl;
    fstream fin(path.c_str());

    vector<string> row;
    string word, temp;

    while (fin >> temp) {

      row.clear();
      stringstream s(temp);
      while (getline(s, word, ',')) {
        row.push_back(word);
      }

      Eigen::Vector3d pos = Eigen::Vector3d(stod(row[0]), stod(row[1]), stod(row[2]));

      geometry_msgs::Point p;
      p.x = pos.x();
      p.y = pos.y();
      p.z = pos.z();

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 0.5;
      line_list.points.push_back(p);

      Eigen::Vector3d point_pose(pos.x(), pos.y(), -stod(row[4])*M_PI/180);

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
    current_index = 0;
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

  void updateControl(tf::Vector3 transf_pos, tf::Quaternion transf_rot, double heading)
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

  Vector3d getNextPoint()
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

      //get robot TF wrt World
      tf::StampedTransform transform;
      try{
        listener.lookupTransform("world", "robot_base",
                                 ros::Time(0), transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      Vector3d ref_point;

      if(finished)
      {
        ref_point = getNextPoint();
        finished=false;
      }

      Eigen::Affine3d robot_pos;

      //Convert from TF to Eigen
      tf::Quaternion transf_rot; transf_rot = transform.getRotation();
      Eigen::Quaterniond robot_rot(transf_rot.w(), transf_rot.x(), transf_rot.y(), transf_rot.z());

      // Ambil posisi Robot wrt World
      tf::Vector3 transf_pos = transform.getOrigin();
      robot_pos.translation() = Eigen::Vector3d(transf_pos.x(), transf_pos.y(), 0);
      robot_pos.linear() = robot_rot.toRotationMatrix();

//      Vector3d point_ref_pos = ref_point.translation();

      Eigen::Matrix3d rot_mat = robot_pos.linear();
//      Vector3d ea = rot_mat.eulerAngles(0,1,2);
//      std::cout << "Angle : "<< ea.y()*180/M_PI << std::endl;

      ref_point.z() = 0;

      // Get ref_point wrt robot
      Eigen::Vector3d e_diff = robot_pos.inverse()*ref_point;

      double phi = atan2(e_diff.y(), e_diff.x());

      std_msgs::Float64 fr_ref_msg;
      std_msgs::Float64 fl_ref_msg;

      if(!stop)
      {
        fl_ref_msg.data = (0.2 + -C*0.8*phi)/(summit_xl_wheel_diameter_*0.5);
        fr_ref_msg.data = (0.2 + C*0.8*phi)/(summit_xl_wheel_diameter_*0.5);
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
//      double err_traj = atan2(e_diff.y(), e_diff.x());

//      Eigen::Vector3d error_traj(e_diff.x(), e_diff.y(), err_traj);

//      Vector3d robot_euler = robot_rot.toRotationMatrix().eulerAngles(0, 1, 2);

//      std::cout << "robot Euler : \n" << robot_euler*(180/M_PI) << std::endl;

//      Matrix3d rot;
//      double c_ref = cos(ref_point.z());
//      double s_ref = sin(ref_point.z());
//      Vector3d err_pos = robot_pos.translation()-Eigen::Vector3d(ref_point.x(), ref_point.y(), 0);
//      rot << c_ref, s_ref, 0,
//             -s_ref, c_ref, 0,
//             0,0,1;

//      Vector3d err_vec = rot*err_pos;

//      err_vec.z() = robot_euler.z() - ref_point.z();


//      updateControl(transf_pos, transf_rot, robot_euler.z());

      std::cout << "getting to target : " << current_index << "\n" << ref_point << std::endl;
      std::cout << "from position : \n" << robot_pos.matrix() << std::endl;
//      std::cout << "Angle Error : " << err_vec.z() << std::endl;
      std::cout << "Error : /n" << e_diff << std::endl;
//      publishSpeed();


      //update next target
      if(e_diff.norm() < 0.1)
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

      marker_pub.publish(points);
      marker_pub.publish(line_strip);

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
