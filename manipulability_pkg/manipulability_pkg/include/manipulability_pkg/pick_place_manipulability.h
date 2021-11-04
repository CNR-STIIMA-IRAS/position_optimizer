#pragma once

#include "manipulability_pkg_msgs/pickPlaceManipulability.h"
#include "manipulability_pkg/trajectory.h"

#include <ros/ros.h>

#include <object_loader_msgs/AddObjects.h>
#include <object_loader_msgs/AttachObject.h>
#include <std_srvs/Trigger.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// #include <moveit/collision_detection_fcl/collision_common.h>
// #include <moveit/collision_detection_fcl/collision_env_fcl.h>  //TODO: mergiare quando sarà possibile

class PickPlaceManipulability
{
  
  std::string PLANNING_GROUP  ;
  std::string GRIP_ENDEFFECTOR;
  std::string BASE_LINK       ;

  bool run_thread_;
  boost::thread*  broadcast_thread_;
  std::vector<geometry_msgs::PoseStamped> broadcast_vec_;
  
  boost::shared_ptr<descartes_moveit::IkFastMoveitStateAdapter> descartes_model_;
  ros::NodeHandle nh;
  rosdyn::ChainPtr chain_bt;
  ros::ServiceClient add_obj_    = nh.serviceClient<object_loader_msgs::AddObjects>   ("add_object_to_scene"  );
  ros::ServiceClient att_obj_    = nh.serviceClient<object_loader_msgs::AttachObject> ("attach_object_to_link");
  ros::ServiceClient remove_obj_ = nh.serviceClient<std_srvs::Trigger>                ("reset_scene"          );
  
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_state::JointModelGroup* joint_model_group_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  robot_model::RobotModelPtr robot_model_; 
  std::shared_ptr< planning_scene::PlanningScene > planning_scene_ ;
  std::shared_ptr< moveit::planning_interface::MoveGroupInterface > move_group_;
  
  std::string object_name_;
  std::string environment_name_;
  std::vector<double> joint_home_;
  std::vector<double> lower_bounds_; 
  std::vector<double> upper_bounds_ ;
  
//   collision_detection::CollisionEnvPtr c_env_;  //TODO mergiare moveit con pradeepr

  
public:
  
  PickPlaceManipulability(ros::NodeHandle nh);
  void broadcastTf();
  void getPlanningScene  (ros::NodeHandle& nh, planning_scene::PlanningScenePtr& ret );
  bool checkObjectsCollisions(std::string obj_1, std::string obj_2);
  bool checkRobotCollisions(std::string object, std::string environment);
  bool checkCollisions();
  bool configScene(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req);
  bool getTrajectory(Trajectory& t, manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req);
  moveit::planning_interface::MoveGroupInterface::Plan generateTwoPointsTraj(std::vector<double> start, std::vector<double> end);
  bool RRTplan(std::vector<double> target_jpos, moveit::planning_interface::MoveGroupInterface::Plan& my_plan, double planning_time, double velocity_factor);
  bool configAndCheck(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req);
  void reset();
  double manipulability(std::vector<double> joints);
  bool planCallback(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req, manipulability_pkg_msgs::pickPlaceManipulabilityResponse& res);
  bool manipulabilityCallback(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req, manipulability_pkg_msgs::pickPlaceManipulabilityResponse& res);
  ~PickPlaceManipulability();
};


