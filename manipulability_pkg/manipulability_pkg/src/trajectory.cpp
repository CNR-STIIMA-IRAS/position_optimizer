#include <manipulability_pkg/trajectory.h>
#include <manipulability_pkg/common.h>
#include <tf/transform_listener.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_planning_helper/manage_trajectories.h>


Trajectory::Trajectory(std::string id, ros::NodeHandle nh, int n_ax)
:id_   (id  )
,nh_   (nh  )
,n_ax_ (n_ax)

{  
  starting_joints.resize(n_ax_);
  target_joints  .resize(n_ax_);  
  
  calculate        = true ;
  check            = true ;
  attach           = false;
  detach           = false;
  allow_collisions = false;
};

bool Trajectory::getParams()
{
  if ( !nh_.getParam ( id_+"/calculate", calculate) )
    ROS_DEBUG( "param %s/%s/calculate not found, default: true",nh_.getNamespace().c_str(),id_.c_str() );
  
  if ( !nh_.getParam ( id_+"/check", check) )
    ROS_DEBUG( "param %s/%s/check not found, default: true",nh_.getNamespace().c_str(),id_.c_str() );
  
  planner = "rrt";
  if ( !nh_.getParam ( id_+"/planner", planner ) )
    ROS_DEBUG( "param %s/%s/planner not found. default RRT",nh_.getNamespace().c_str(),id_.c_str() );
    
  if ( !nh_.getParam ( id_+"/is_offset_pose", is_offset_pose) )
  {
    is_offset_pose = false;
    ROS_DEBUG ( "param %s/%s/is_offset_pose not found. default false",nh_.getNamespace().c_str(),id_.c_str() );
  }
  if(is_offset_pose)
  {
    if ( !nh_.getParam ( id_+"/offset_wrt_pose", offset_wrt) )
    {
      ROS_ERROR ( "param %s/%s/offset_wrt_pose not found. return",nh_.getNamespace().c_str(),id_.c_str() );
      return false;
    }
    if ( !nh_.getParam ( id_+"/offset", offset) )
    {
      ROS_ERROR ( "param %s/%s/offset not found. return",nh_.getNamespace().c_str(),id_.c_str());
      return false;
    }
    if(offset.size() != 6)
    {
      ROS_ERROR ( "param %s/offset size is %zu , wanted size of 6. return",id_.c_str(), offset.size() );
      return false;
    }
    offset[3] *= M_PI/180;
    offset[4] *= M_PI/180;
    offset[5] *= M_PI/180;
    preoffset        = false;
    if ( !nh_.getParam ( id_+"/pre_offset", preoffset) )
      ROS_DEBUG ( "param %s%s/pre_offset not found, default: false",nh_.getNamespace().c_str(),id_.c_str() );
  }
  
  if ( !nh_.getParam ( id_+"/planning_time", planning_time) )
    ROS_DEBUG ( "param %s%s/planning_time not found, default: 10.0",nh_.getNamespace().c_str(),id_.c_str() );
  if ( !nh_.getParam ( id_+"/planning_velocity", planning_velocity) )
    ROS_DEBUG ( "param %s%s/planning_velocity not found, default: 0.1",nh_.getNamespace().c_str(),id_.c_str());
  if ( !nh_.getParam ( id_+"/attach", attach) )
    ROS_DEBUG ( "param %s%s/attach not found, default: false",nh_.getNamespace().c_str(),id_.c_str());
  if ( !nh_.getParam ( id_+"/detach", detach) )
    ROS_DEBUG ( "param %s%s/detach not found, default: false",nh_.getNamespace().c_str(),id_.c_str() );
  if ( !nh_.getParam ( id_+"/allow_collisions", allow_collisions) )
    ROS_DEBUG ( "param %s%s/allow_collisions not found, default: false",id_.c_str(), nh_.getNamespace().c_str() );
  if ( !nh_.getParam ( id_+"/seed_starting_joints", seed_starting_joints) )
  {
    seed_starting_joints = std::vector<double>(n_ax_,0);
    ROS_DEBUG_STREAM("seed_starting_joints default zero");
  }
  if ( !nh_.getParam ( id_+"/seed_target_joints", seed_target_joints) )
  {
    seed_target_joints = std::vector<double>(n_ax_,0);
    ROS_DEBUG_STREAM("seed_target_joints default zero");
  }
  if ( !nh_.getParam ( id_+"/starting_joints", starting_joints) )
    ROS_DEBUG ( "param %s%s/starting_joints not found",id_.c_str(), nh_.getNamespace().c_str() );
  if ( !nh_.getParam ( id_+"/target_joints", target_joints) )
    ROS_DEBUG ( "param %s%s/target_joints not found",id_.c_str(), nh_.getNamespace().c_str() );
  
  return true;
}

geometry_msgs::Pose Trajectory::getPoseFromJoints(rosdyn::Chain& chain_bt_, std::vector<double> &vec )
{
  geometry_msgs::Pose ret;
  
  if(vec.size() != n_ax_)
    ROS_ERROR_STREAM("[Trajectory::getPoseFromJoints]: size of vector passed ("<< vec.size() <<") is different with respect to n_ax ("<< n_ax_ <<")");
  
  Eigen::VectorXd qs(n_ax_);
  for (int i=0; i<n_ax_; i++)
    qs[i] = vec[i];
  
  Eigen::Affine3d Ts = chain_bt_.getTransformation(qs);
  tf::poseEigenToMsg(Ts, ret);
  return ret;
}

void Trajectory::setStartingPoseFromStartingJoints(rosdyn::Chain& chain_bt_ )
{
  starting_pose = getPoseFromJoints(chain_bt_,starting_joints);
  return;
}

void Trajectory::setStartingPoseFromJoints(rosdyn::Chain& chain_bt_, std::vector<double>& vec)
{
  starting_joints = vec;
  starting_pose = getPoseFromJoints(chain_bt_,starting_joints);
  return;
}

void Trajectory::setTargetPoseFromTargetJoints(rosdyn::Chain& chain_bt_ )
{
  target_pose = getPoseFromJoints(chain_bt_, target_joints);
  return;
}

void Trajectory::setTargetPoseFromJoints(rosdyn::Chain& chain_bt_, std::vector<double>& vec)
{
  target_joints = vec;
  target_pose = getPoseFromJoints(chain_bt_, target_joints);
  return;
}

std::vector<std::vector<double>> Trajectory::getIKsolutions(descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose& pose)
{
  std::vector< std::vector<double> > joints;
  
  Eigen::Affine3d Twp;
  tf::poseMsgToEigen(pose,Twp);
  
  descartes_core::Frame frame(Twp);
  descartes_trajectory::CartTrajectoryPt p(frame);
  p.getJointPoses( descartes_model ,joints);
  
  return joints;
}

std::vector<double> Trajectory::getJointsFromPose(descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose& pose, std::vector<double> seed)
{
  if( seed.size() == 0 )
    seed.resize(n_ax_);
  else if (seed.size() != n_ax_)
    ROS_ERROR_STREAM("[Trajectory::getJointsFromPose]: size of seed vector passed ("<< seed.size() <<") is different with respect to n_ax ("<< n_ax_ <<")");
  
  std::vector<double> ret;
  Eigen::VectorXd distance (n_ax_);
  
  std::vector< std::vector<double> > joints = getIKsolutions(descartes_model, pose);
  
  if (joints.empty())
  {
    ROS_ERROR("no solutions found for this pose/seeds. returning empty vector");
    ret = {};
    return ret;
  }
  
  std::vector<double> min_distance;
  for(size_t j = 0; j<joints.size(); j++)
  {
    for(size_t i = 0; i<n_ax_; i++)
      distance[i] = std::fabs( seed[i] - joints[j][i]);  
    
    min_distance.push_back(distance.norm());
  }
  int idx = std::min_element(min_distance.begin(),min_distance.end()) - min_distance.begin();
  ret = joints[idx];
  
  return ret;   
}

bool Trajectory::setStartingJointsFromStartingPose(descartes_moveit::IkFastMoveitStateAdapter &descartes_model)
{
  starting_joints = getJointsFromPose(descartes_model,starting_pose, seed_starting_joints);
  if(starting_joints.empty())
    return false;
  return true; 
}

bool Trajectory::setStartingJointsFromPose(descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose pose)
{
  starting_pose = pose;
  return setStartingJointsFromStartingPose(descartes_model);
}

bool Trajectory::setTargetJointsFromTargetPose(descartes_moveit::IkFastMoveitStateAdapter &descartes_model)
{
  target_joints = getJointsFromPose(descartes_model,target_pose, seed_target_joints);
  if(target_joints.empty())
    return false;
  return true; 
}

bool Trajectory::setTargetJointsFromPose(descartes_moveit::IkFastMoveitStateAdapter &descartes_model, geometry_msgs::Pose pose)
{
  target_pose = pose;
  return setTargetJointsFromTargetPose(descartes_model);
}


bool Trajectory::setOffsetTargetPose()
{
  
  if (!is_offset_pose)
  {
    ROS_ERROR_STREAM("the pose is not defined as an offset pose! return false");
    return false;
  }
  
  ROS_DEBUG_STREAM("listening to tf from /iiwa_link_0 to "<<offset_wrt);
  tf::StampedTransform T_robot_base_panel;
  tf::TransformListener listener;
  listener.waitForTransform ( "iiwa_link_0", offset_wrt, ros::Time ( 0 ), ros::Duration ( 10.0 ) );
  listener.lookupTransform  ( "iiwa_link_0", offset_wrt, ros::Time ( 0 ), T_robot_base_panel);
  
  geometry_msgs::Pose pp;
  tf::poseTFToMsg(T_robot_base_panel,pp);
  
  target_pose = common::getOffsetPose(pp, offset, preoffset);
  
  return true;
}

bool Trajectory::getPlannedTrajectoryFromParam()
{
  ROS_DEBUG_STREAM("looking for trajectory: "<<id_<<" on rosparam");
  std::string path = "trajectories_planned/"+id_;
  if(! trajectory_processing::getTrajectoryFromParam(nh_, path, plan.trajectory_.joint_trajectory))
  {
    ROS_ERROR("%s not found ros param, nh ns: [%s]",id_.c_str(), nh_.getNamespace().c_str());
    return false;
  }
  return true;
}

bool Trajectory::setPlannedTrajectoryToParam()
{
  ROS_DEBUG_STREAM("saving trajectory: "<<id_<<" to rosparam");
    std::string path = "trajectories_planned/"+id_;
    if(! trajectory_processing::setTrajectoryToParam(nh_, path, plan.trajectory_.joint_trajectory))
    {
      ROS_ERROR("%s not set on ros param",id_.c_str());
      return false;
    }
  return true;
}
