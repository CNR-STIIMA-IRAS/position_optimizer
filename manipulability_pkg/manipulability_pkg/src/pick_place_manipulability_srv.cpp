#include <ros/ros.h>
#include <manipulability_pkg/pick_place_manipulability.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place_manipulability_srv");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  
  PickPlaceManipulability man(nh);
  ros::ServiceServer manip_service = nh.advertiseService("pick_place_manipulability_srv", &PickPlaceManipulability::manipulabilityCallback, &man);
  ros::ServiceServer plan_service  = nh.advertiseService("pick_place_plan_srv"          , &PickPlaceManipulability::planCallback          , &man);
  
  ros::waitForShutdown();
  
  return 0;
}


