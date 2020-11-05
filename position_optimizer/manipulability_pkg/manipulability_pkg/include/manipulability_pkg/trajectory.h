#pragma once

#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <descartes_moveit/ikfast_moveit_state_adapter.h>
#include <rosdyn_core/primitives.h>

class Trajectory 
{
protected:
  std::vector<double> starting_joints;
  std::vector<double> target_joints  ;
  
  geometry_msgs::Pose starting_pose;
  geometry_msgs::Pose target_pose  ;
  
public:
  
  std::string id_;
  ros::NodeHandle nh_;
  int n_ax_;
  
  std::string planner;
  std::string offset_wrt;
  
  bool calculate       ;
  bool check           ;
  bool attach          ;
  bool detach          ;
  bool allow_collisions;
  bool preoffset       ;
  bool is_offset_pose  ;
  
  double planning_time     = 10;
  double planning_velocity = 0.1;
  
  std::vector<double> seed_starting_joints;
  std::vector<double> seed_target_joints;
  std::vector<double> offset            ;
  
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  
  Trajectory(std::string id, ros::NodeHandle nh, int n_ax);
  
  bool getParams();
  
  geometry_msgs::Pose getPoseFromJoints (rosdyn::Chain& chain_bt_, std::vector<double> &vec );
  void setStartingPoseFromJoints        (rosdyn::Chain& chain_bt_, std::vector<double>& vec);
  void setTargetPoseFromJoints          (rosdyn::Chain& chain_bt_, std::vector<double>& vec);
  void setStartingPoseFromStartingJoints(rosdyn::Chain& chain_bt_);
  void setTargetPoseFromTargetJoints    (rosdyn::Chain& chain_bt_);
  
  std::vector<std::vector<double>> getIKsolutions     (descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose& pose);
  std::vector<double> getJointsFromPose               (descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose& pose, std::vector<double> seed = std::vector<double>());
  bool setStartingJointsFromPose                      (descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose pose);
  bool setTargetJointsFromPose                        (descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose pose);
  bool setStartingJointsFromStartingPose              (descartes_moveit::IkFastMoveitStateAdapter &descartes_model);
  bool setTargetJointsFromTargetPose                  (descartes_moveit::IkFastMoveitStateAdapter &descartes_model);
  bool setOffsetTargetPose();
  
  bool getPlannedTrajectoryFromParam();
  bool setPlannedTrajectoryToParam();
  
  inline void setStartingJoints(std::vector<double>& vec){starting_joints = vec;};
  inline std::vector<double> getStartingJoints(){return starting_joints;};
  inline void setTargetJoints  (std::vector<double>& vec){target_joints   = vec;};
  inline std::vector<double> getTargetJoints  (){return target_joints;};
  
  inline void setStartingPose(geometry_msgs::Pose pose){starting_pose = pose;};
  inline geometry_msgs::Pose getStartingPose(){return starting_pose;};
  inline void setTargetPose(geometry_msgs::Pose pose){target_pose= pose;};
  inline geometry_msgs::Pose getTargetPose(){return target_pose;};
  
};



