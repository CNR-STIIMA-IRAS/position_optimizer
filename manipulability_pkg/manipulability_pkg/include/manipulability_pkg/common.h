#pragma once
#include <fstream>
#include <tf/tf.h>
#include <ros/ros.h>

static const char* DEFAULT      = "\033[0m";
static const char* RESET        = "\033[0m";
static const char* BLACK        = "\033[30m";
static const char* RED          = "\033[31m";
static const char* GREEN        = "\033[32m";
static const char* YELLOW       = "\033[33m";
static const char* BLUE         = "\033[34m";
static const char* MAGENTA      = "\033[35m";
static const char* CYAN         = "\033[36m";
static const char* WHITE        = "\033[37m";
static const char* BOLDBLACK    = "\033[1m\033[30m";
static const char* BOLDRED      = "\033[1m\033[31m";
static const char* BOLDGREEN    = "\033[1m\033[32m";
static const char* BOLDYELLOW   = "\033[1m\033[33m";
static const char* BOLDBLUE     = "\033[1m\033[34m";
static const char* BOLDMAGENTA  = "\033[1m\033[35m";
static const char* BOLDCYAN     = "\033[1m\033[36m";
static const char* BOLDWHITE    = "\033[1m\033[37m";

static const std::string CARGO_UPPER_POSE_TF = "/tf_cargo_upper" ;
static const std::string CARGO_OFFSET_TF     = "/tf_cargo_offset";

static const std::string IIWA_ERROR_NS = "/iiwa/error_msg";


void vecToTf ( std::vector<double> *pose ,tf::Pose& transform )
{

  geometry_msgs::Pose p;
  p.position.x = pose->at ( 0 );
  p.position.y = pose->at ( 1 );
  p.position.z = pose->at ( 2 );

  tf::Quaternion q;
  if ( pose->size() == 6 )
  {
    q = tf::createQuaternionFromRPY ( pose->at ( 3 ),pose->at ( 4 ),pose->at ( 5 ) );
    tf::quaternionTFToMsg ( q, p.orientation );
  }
  else
    q = tf::Quaternion ( pose->at ( 4 ),pose->at ( 5 ),pose->at ( 6 ),pose->at ( 3 ) );

  tf::poseMsgToTF ( p, transform );
}
void vecToTf ( std::vector<double> pose ,tf::Pose& transform )
{

  geometry_msgs::Pose p;
  p.position.x = pose.at ( 0 );
  p.position.y = pose.at ( 1 );
  p.position.z = pose.at ( 2 );

  tf::Quaternion q;
  if ( pose.size() == 6 )
  {
    q = tf::createQuaternionFromRPY ( pose.at ( 3 ),pose.at ( 4 ),pose.at ( 5 ) );
    tf::quaternionTFToMsg ( q, p.orientation );
  }
  else
    q = tf::Quaternion ( pose.at ( 4 ),pose.at ( 5 ),pose.at ( 6 ),pose.at ( 3 ) );

  tf::poseMsgToTF ( p, transform );
}


namespace common
{
  
geometry_msgs::Pose vecToPose(std::vector<double> pose, bool deg = true){
  
  geometry_msgs::Pose ret;
  tf::Pose transform;
  
  tf::Vector3 v = tf::Vector3(pose.at(0),pose.at(1),pose.at(2));
  tf::Quaternion q;
  
  if(pose.size() == 6)
  {
    if (deg)
    {
      pose.at(3) = pose.at(3) * M_PI / 180.0;
      pose.at(4) = pose.at(4) * M_PI / 180.0;
      pose.at(5) = pose.at(5) * M_PI / 180.0;
    }
    q = tf::createQuaternionFromRPY(pose.at(3),pose.at(4),pose.at(5)); 
  }
  else
    q = tf::Quaternion(pose.at(4),pose.at(5),pose.at(6),pose.at(3)); 
  
  transform.setOrigin(v);
  transform.setRotation(q); 
  
  tf::poseTFToMsg(transform, ret);
  
  return ret;
} 

geometry_msgs::Pose getOffsetPose(geometry_msgs::Pose pose, std::vector<double> offset, bool pre_offset = false)
{
  geometry_msgs::Pose ret; 
  tf::Pose pose_tf, offset_tf;
  tf::poseMsgToTF(pose,pose_tf);
  vecToTf(offset,offset_tf);
  if(pre_offset)
    pose_tf = offset_tf * pose_tf;
  else
    pose_tf = pose_tf * offset_tf;
  tf::poseTFToMsg(pose_tf, ret);
  return ret;
}

void setTfAsParam(ros::NodeHandle nh,tf::Pose t, std::string s)
{
  geometry_msgs::Pose p;
  tf::poseTFToMsg(t,p);
  
  std::vector<double> v = {p.position.x,
                           p.position.y,
                           p.position.z,
                           p.orientation.x,
                           p.orientation.y,
                           p.orientation.z,
                           p.orientation.w };
  nh.setParam(s,v);
}
bool getTfFromParam(ros::NodeHandle nh, tf::Pose& t, std::string s)
{
  std::vector<double> v;
  if(!nh.getParam(s,v))
  {
    ROS_ERROR_STREAM(s<<" not found on rosparam on ns "<<nh.getNamespace());
    return false;
  }
  
  geometry_msgs::Pose p;
  
  p.position.x    = v[0] ;
  p.position.y    = v[1] ;
  p.position.z    = v[2] ;
  p.orientation.x = v[3] ;
  p.orientation.y = v[4] ;
  p.orientation.z = v[5] ;
  p.orientation.w = v[6] ;
  
  tf::poseMsgToTF(p, t);
  
  return true;
}


}


template< typename T > std::string vec_to_string  ( const std::vector< T >& v
                                              , const std::string prefix = "["
                                              , const std::string delimeter = ", "
                                              , const std::string terminator ="]" )
{
  std::string ret = prefix;
  if(v.size() == 0)
    return "";
  
  for( size_t i=0; i < v.size()-1; i++)
    ret += std::to_string( v[i] ) + delimeter;
  ret += std::to_string( v.back() ) + terminator;
  
  return ret;
}
