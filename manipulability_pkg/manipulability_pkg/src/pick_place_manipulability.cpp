#include "manipulability_pkg/pick_place_manipulability.h"
#include "manipulability_pkg/common.h"

#include <rosparam_utilities/rosparam_utilities.h>
#include <descartes_planner/dense_planner.h>
#include <tf/transform_broadcaster.h>


  PickPlaceManipulability::PickPlaceManipulability(ros::NodeHandle nh)
  :nh(nh)
  {
    if ( !nh.getParam ( "planning_group", PLANNING_GROUP) )
    {
        ROS_ERROR ( "posizione %s/planning_group", nh.getNamespace().c_str() );
        return;
    }
    
    if ( !nh.getParam ( "base_link", BASE_LINK) )
    {
        ROS_ERROR ( "posizione %s/base_link", nh.getNamespace().c_str() );
        return;
    }
    
    if ( !nh.getParam ( "tip_link", GRIP_ENDEFFECTOR) )
    {
        ROS_ERROR ( "posizione %s/tip_link", nh.getNamespace().c_str() );
        return;
    }
    if ( !nh.getParam ( "object_name", object_name_) )
    {
        ROS_ERROR ( "posizione %s/object_name", nh.getNamespace().c_str() );
        return;
    }
    if ( !nh.getParam ( "environment_name", environment_name_) )
    {
        ROS_ERROR ( "posizione %s/environment_name", nh.getNamespace().c_str() );
        return ;
    }
    if ( !nh.getParam ( "joint_home", joint_home_) )
    {
        ROS_ERROR ( "posizione %s/joint_home", nh.getNamespace().c_str() );
        std::cin.get();
        return ;
    }
    if ( !nh.getParam ( "upper_bounds", upper_bounds_) )
    {
        ROS_ERROR ( "posizione %s/upper_bounds", nh.getNamespace().c_str() );
        std::cin.get();
        return ;
    }
    if ( !nh.getParam ( "lower_bounds", lower_bounds_) )
    {
        ROS_ERROR ( "posizione %s/lower_bounds", nh.getNamespace().c_str() );
        std::cin.get();
        return ;
    }
    
    // initialize descartes model
    descartes_model_.reset( ( new descartes_moveit::IkFastMoveitStateAdapter ));  
    nh.setParam ( PLANNING_GROUP + "/ikfast_base_frame",BASE_LINK );
    nh.setParam ( PLANNING_GROUP + "/ikfast_tool_frame",GRIP_ENDEFFECTOR );
    if ( !descartes_model_->initialize ( "robot_description", PLANNING_GROUP, BASE_LINK, GRIP_ENDEFFECTOR ) )
    {
      ROS_INFO ( "Could not initialize robot model" );
    }
    descartes_planner::DensePlanner descartes_planner;
    descartes_planner.initialize ( ( descartes_core::RobotModelPtr ) descartes_model_ );
    
    urdf::Model urdf_model;
    if (!urdf_model.initParam("robot_description"))
    {
      ROS_ERROR("Urdf robot_description '%s' does not exist",(nh.getNamespace()+"/robot_description").c_str());
    }
    Eigen::Vector3d gravity;
    gravity << 0, 0, -9.806;
    chain_bt = rosdyn::createChain(urdf_model, BASE_LINK, GRIP_ENDEFFECTOR, gravity);
    
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface (PLANNING_GROUP));
    ROS_INFO_STREAM("waiting for service: "<< add_obj_.getService());
    add_obj_.waitForExistence();
    ROS_INFO_STREAM("waiting for service: "<< remove_obj_.getService());
    remove_obj_.waitForExistence();
    
    robot_model_loader_.reset( new robot_model_loader::RobotModelLoader ( "robot_description" ) );
    planning_scene_monitor_.reset( new planning_scene_monitor::PlanningSceneMonitor( robot_model_loader_ ) ); 
    planning_scene_monitor_->startSceneMonitor( );
    planning_scene_monitor_->startWorldGeometryMonitor( );
    planning_scene_monitor_->startStateMonitor( );
    robot_model_ = robot_model_loader::RobotModelLoader ( "robot_description" ).getModel();
    joint_model_group_ = robot_model_->getJointModelGroup("manipulator");
    planning_scene_.reset (new planning_scene::PlanningScene( robot_model_ ));
//     c_env_.reset(new collision_detection::CollisionEnvFCL(robot_model_)); //TODO mergiare moveit con pradeepr
    
    run_thread_ = true;
    broadcast_thread_ = new boost::thread(&PickPlaceManipulability::broadcastTf,this);
    
  }
  
  PickPlaceManipulability::~PickPlaceManipulability()
  {
    run_thread_ = false;
    broadcast_thread_->join();
  }
  
  void PickPlaceManipulability::broadcastTf()
  {
    ros::Rate r(50);
    tf::TransformBroadcaster br;
    while(ros::ok() && run_thread_)
    {
      for (auto p : broadcast_vec_)
      {
        tf::Transform t;
        tf::poseMsgToTF(p.pose,t);
        
        br.sendTransform(tf::StampedTransform(t, ros::Time::now(), BASE_LINK, p.header.frame_id));
      }
      r.sleep();
    }
  }

  void PickPlaceManipulability::getPlanningScene   ( ros::NodeHandle& nh
                          , planning_scene::PlanningScenePtr& ret )
  {
    ros::ServiceClient planning_scene_service;
    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene> ( "get_planning_scene" );
    if ( !planning_scene_service.waitForExistence ( ros::Duration ( 5.0 ) ) )
    {
        ROS_ERROR ( "getPlanningScene Failed: service '%s/get_planning_scene' does not exist", nh.getNamespace().c_str() );
    }

    moveit_msgs::PlanningScene planning_scene_msgs;

    {
        /// ROBOT_STATE
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.ROBOT_STATE;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }


        planning_scene_msgs.name = response.scene.name;
        planning_scene_msgs.robot_state = response.scene.robot_state;
    }
    {
        // WORLD_OBJECT_GEOMETRY && WORLD_OBJECT_NAMES
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.WORLD_OBJECT_GEOMETRY;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.world.collision_objects = response.scene.world.collision_objects;
    }
    {
        // OCTOMAP
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.OCTOMAP;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.world.octomap = response.scene.world.octomap;
    }
    {
        // TRANSFORMS
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.TRANSFORMS;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.fixed_frame_transforms = response.scene.fixed_frame_transforms;
    }
    {
        // ALLOWED_COLLISION_MATRIX
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;
        request.components.components = request.components.ALLOWED_COLLISION_MATRIX;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.allowed_collision_matrix = response.scene.allowed_collision_matrix;
    }
    {
        // LINK_PADDING_AND_SCALING
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.LINK_PADDING_AND_SCALING;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.link_padding = response.scene.link_padding;
        planning_scene_msgs.link_scale   = response.scene.link_scale;
    }
    {
        // OBJECT_COLORS
        moveit_msgs::GetPlanningScene::Request request;
        moveit_msgs::GetPlanningScene::Response response;

        request.components.components = request.components.LINK_PADDING_AND_SCALING;
        if ( !planning_scene_service.call ( request, response ) )
        {
            ROS_WARN ( "Could not call planning scene service to get object names" );
        }
        planning_scene_msgs.object_colors = response.scene.object_colors;
    }

    ret->setPlanningSceneMsg ( planning_scene_msgs );

    return;
  }
  
  
  bool PickPlaceManipulability::checkObjectsCollisions(std::string obj_1, std::string obj_2)
  {
    
    getPlanningScene ( nh, planning_scene_ );
    
    std::vector<std::string> object_group1 = { obj_1 };
    std::vector<std::string> object_group2 = { obj_2 };
    
    //TODO:: mergiare con moveit pradeepr
    
//     bool collision = planning_scene_->getCollisionEnv()->checkCollisionBetweenObjectGroups(object_group2, object_group1);
//     
//     if(collision)
//     {
//       ROS_ERROR_STREAM("Collision found between "+obj_1+" and "+obj_2);
//       return false;
//     } 
    return true;
  }
  
  bool PickPlaceManipulability::checkRobotCollisions(std::string object, std::string environment)
  {
    planning_scene_monitor::LockedPlanningSceneRW ps = planning_scene_monitor::LockedPlanningSceneRW( planning_scene_monitor_ ); 
    robot_state::RobotState robot_state = ps->getCurrentStateNonConst();
    collision_detection::AllowedCollisionMatrix *acm = &ps->getAllowedCollisionMatrixNonConst();
    
    //TODO rimuovere
// // // //     uncomment for cargo
    acm->setEntry("camera",environment,true);
    acm->setEntry("tank",environment,true);
// // // //     uncomment for cargo
    
    getPlanningScene ( nh, planning_scene_ );
    
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult  collision_result;  
    
    robot_state.updateCollisionBodyTransforms();
    collision_request.contacts = true;
    collision_request.max_contacts = 100;
    collision_request.max_contacts_per_pair = 5;
    collision_request.verbose = false;
    
    collision_result.clear();
    planning_scene_->checkCollision(collision_request, collision_result, robot_state, *acm);
    
    if(collision_result.contact_count != 0)
    {
      ROS_DEBUG("collision found");
      for(auto& p : collision_result.contacts)
        ROS_ERROR_STREAM( "Contact detected between: "<<p.first.first.data()<<" and "<<p.first.second.data());
      return false;
    } 
    return true;
  }
  
  bool PickPlaceManipulability::checkCollisions()
  {
    std::string object = object_name_;
    object +="_0";
    std::string environment = environment_name_;
    environment +="_0";
    
    // collisioni tra oggetti
    
    if(!checkObjectsCollisions(object,environment))
    {
      ROS_ERROR_STREAM("collision between two collision objects found");
      return false;
    }
    
    // collisioni tra oggetti e robot
    
    if(!checkRobotCollisions(object,environment))
    {
      ROS_ERROR_STREAM("collision between robot and objects found");
      return false;
    }
    
    return true;
  }
  
  bool PickPlaceManipulability::configScene(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req)
  {
    std_srvs::Trigger rem_srv;
    remove_obj_.call(rem_srv);
    
    object_loader_msgs::AddObjects srv;
    
    {
      object_loader_msgs::Object obj;
      
      obj.object_type=object_name_;
      
      obj.pose.pose = req.pick_pose;
      obj.pose.header.frame_id = BASE_LINK;
      srv.request.objects.push_back(obj);
    }
    {
      object_loader_msgs::Object obj;
      
      obj.object_type=environment_name_;
      
      //TODO eliminare con violenza
      
      std::vector<double> environmenta_offset_vec_;
      if ( !nh.getParam ( "place_pose/offset", environmenta_offset_vec_ ) )
      {
          ROS_ERROR ( "posizione %s/place_pose/offset non caricata", nh.getNamespace().c_str() );
          return false;
      }
      tf::Pose attach_point_to_fuselage_origin; vecToTf ( environmenta_offset_vec_, attach_point_to_fuselage_origin );
      
      tf::Pose tf_pos;
      tf::poseMsgToTF(req.place_pose,tf_pos);
      geometry_msgs::Pose fuselage_pose; 
      tf::poseTFToMsg( tf_pos * attach_point_to_fuselage_origin.inverse(), fuselage_pose);
      obj.pose.pose = fuselage_pose;
      //
      
//       obj.pose.pose = req.place_pose;  //TODO scommentare quando torneranno tempi migliori
      obj.pose.header.frame_id = BASE_LINK;
      srv.request.objects.push_back(obj);
    }
    
    add_obj_.call(srv);
    
    broadcast_vec_.clear();
    {
      geometry_msgs::PoseStamped p;
      p.pose = req.pick_pose;
      p.header.frame_id = req.pick_name;
      broadcast_vec_.push_back(p);
    }
    {
      geometry_msgs::PoseStamped p;
      p.pose = req.place_pose;
      p.header.frame_id = req.place_name;
      broadcast_vec_.push_back(p);
    }
    
    return true;
  }
    
  bool PickPlaceManipulability::getTrajectory(Trajectory& t, manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req)
  {
    if(!t.getParams())
    {
      ROS_ERROR ( "error in getting params" );
      return false;
    }
    
    std::vector<std::vector<double> > seeds;
    
    std::vector<double> vvv;
    
    if (t.id_ == req.pick_name)
    {
      if (req.seed_pick_joints.size()>1)
      {
        for (auto c : req.seed_pick_joints)
          vvv.push_back(c);
        
        t.seed_target_joints = vvv;
      }
    }
    else
    {
      if (req.seed_place_joints.size()>1)
      {
        for (auto c : req.seed_place_joints)
          vvv.push_back(c);
        
        t.seed_target_joints = vvv;
      }      
    }
    seeds.push_back(t.seed_target_joints);
    
    descartes_model_->setSeedStates(seeds);
    
    std::vector<double> offset;
    tf::Pose tf_pos;
    
    if (t.id_ == req.pick_name)
    {
      if ( !nh.getParam ( "pick_pose/ee_offset", offset ))
      {
          ROS_ERROR ( "posizione %s/place_pose/offset non caricata", nh.getNamespace().c_str() );
          return false;
      }
      tf::poseMsgToTF(req.pick_pose,tf_pos);
    }
    else
    {
      if ( !nh.getParam ( "place_pose/ee_offset", offset ))
      {
          ROS_ERROR ( "posizione %s/place_pose/offset non caricata", nh.getNamespace().c_str() );
          return false;
      }
      tf::poseMsgToTF(req.place_pose,tf_pos);
    }
    
    tf::Pose tf_off; vecToTf ( offset, tf_off);
    
    geometry_msgs::Pose pose; 
    tf::poseTFToMsg( tf_pos * tf_off, pose);
    
    t.setTargetPose(pose);
    
    geometry_msgs::PoseStamped ppp;
    ppp.pose = pose;
    std::string str = t.id_; str+="_target";
    ppp.header.frame_id = str;
    broadcast_vec_.push_back(ppp);
    
    if(!t.setTargetJointsFromTargetPose(*descartes_model_))
    {
      ROS_FATAL_STREAM("unable to compute IK solution for "+t.id_+"  pose");
      return false;
    }
    return true;
  }
  
  moveit::planning_interface::MoveGroupInterface::Plan PickPlaceManipulability::generateTwoPointsTraj(std::vector<double> start, std::vector<double> end)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    trajectory_msgs::JointTrajectoryPoint ps,pe; 
    plan.trajectory_.joint_trajectory.joint_names = move_group_->getJointNames();
    ps.positions = start;
    ps.time_from_start = ros::Duration(0.0);
    pe.positions = end;
    pe.time_from_start = ros::Duration(0.01);
    plan.trajectory_.joint_trajectory.points.push_back(ps);
    plan.trajectory_.joint_trajectory.points.push_back(pe);
    
    return plan;
  }
  
  bool PickPlaceManipulability::RRTplan(std::vector<double> target_jpos, moveit::planning_interface::MoveGroupInterface::Plan& my_plan, double planning_time, double velocity_factor)
  {
    
    move_group_->setJointValueTarget         ( target_jpos );
    move_group_->setPlanningTime             (planning_time);
    move_group_->setMaxVelocityScalingFactor (velocity_factor);
    move_group_->setStartStateToCurrentState();
    
    ROS_DEBUG("planning with %s", move_group_->getPlannerId().c_str());
    
    moveit::planning_interface::MoveItErrorCode err = move_group_->plan ( my_plan );
    try
    {
      if ( (err != moveit::planning_interface::MoveItErrorCode::SUCCESS) )
      {
        ROS_ERROR("planning error !! ");
        return false;
      }
    }
    catch(const moveit::Exception e )
    {
      ROS_INFO_STREAM(e.what());
    }
    catch(std::exception e)
    {
      ROS_INFO_STREAM(e.what());
    }
    catch(...)
    {
      ROS_INFO("IDK");
    }
    
    return true;
  }
  
  bool PickPlaceManipulability::configAndCheck(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req)
  {
    if (!configScene(req))
    {
      ROS_ERROR("error in scene configuration . return");
      return false;
    }
    
    if(!checkCollisions())
    {
      ROS_ERROR("error in scene check collisions. return");
      return false;
    }
    return true;
  }
  
  void PickPlaceManipulability::reset()
  {
    move_group_->execute( generateTwoPointsTraj(move_group_->getCurrentJointValues(), joint_home_));
  }
  
  
  double PickPlaceManipulability::manipulability(std::vector<double> joints)
  {
    Eigen::VectorXd j;
    
    j.resize(joints.size());
    for (int i=0; i<joints.size(); i++)
      j(i) = joints[i];
    
    Eigen::Matrix6Xd  J_b = chain_bt->getJacobian (j);
    
    std::vector<double> prod;
    
    for (auto j:joints)
      ROS_DEBUG_STREAM("j: "<<j);
    
    for (int i=0; i<joints.size(); i++)
    {
      double num = (joints[i] - lower_bounds_[i])*(upper_bounds_[i] - joints[i]); 
      double den = upper_bounds_[i] - lower_bounds_[i];
      prod.push_back( num / pow(den,2.0));
    }
    double min_prod = *std::min_element(prod.begin(),prod.end());
    double k = 100;
    double penalty = 1 - exp(-k * min_prod);
    
    ROS_DEBUG_STREAM("mu: "<<(std::sqrt((J_b * J_b.transpose()).determinant())));
    ROS_DEBUG_STREAM("penal: "<<penalty);
    
    return (std::sqrt((J_b * J_b.transpose()).determinant())) * penalty;
  }
  
  
  bool PickPlaceManipulability::planCallback(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req, manipulability_pkg_msgs::pickPlaceManipulabilityResponse& res)
  {
    
    if (!configAndCheck(req))
    {
      res.success = false;
      reset();
      return true;
    }
    
    ROS_DEBUG_STREAM("id: "<<req.place_name);
    
    
    Trajectory t_pick(req.pick_name, nh, joint_home_.size()) ;
    if (!getTrajectory(t_pick, req)) 
    {
      res.success = false;
      reset();
      return true;
    }
    
    Trajectory t_place(req.place_name, nh, joint_home_.size()) ;
    if (!getTrajectory(t_place, req)) 
    {
      ROS_ERROR ( "error in getting trajectory" );
      res.success = false;
      reset();
      return true;
    }
    
    move_group_->execute( generateTwoPointsTraj(move_group_->getCurrentJointValues(), t_place.getTargetJoints()));
    
    if(req.precomputed_path)
    {
      if (t_place.getPlannedTrajectoryFromParam())
      {
        ROS_DEBUG_STREAM("setting trajectory "+t_place.id_+ " to param");
        std::vector<std::vector<double>> planned_path;
        for (auto js : t_place.plan.trajectory_.joint_trajectory.points)
          planned_path.push_back(js.positions);
        
        planned_path.erase(planned_path.begin());
        
        rosparam_utilities::setParamNum(nh,"/preload_path",planned_path,3);
        
        nh.setParam("/preload_radius", 0.07);
      }
      else
        ROS_WARN_STREAM("trajectory "+t_place.id_+" not found");
    }
    else
      nh.deleteParam("/preload_path");
    
    move_group_->execute( generateTwoPointsTraj(move_group_->getCurrentJointValues(), t_pick.getTargetJoints()));
    
    std::string object = object_name_; object +="_0";
    std::string environment = environment_name_; environment +="_0";
    
    object_loader_msgs::AttachObject ob;
    ob.request.obj_id = object;
    ob.request.link_name = GRIP_ENDEFFECTOR;
    
    ros::Duration(0.1).sleep();
    att_obj_.call(ob);
    
    getPlanningScene ( nh, planning_scene_ );
    
    if( !RRTplan( t_place.getTargetJoints(), t_place.plan, t_place.planning_time, t_place.planning_velocity ) )
    {
      ROS_ERROR("%s: RRTplan failed!", t_place.id_.c_str());
      if(!t_place.setTargetJointsFromTargetPose(*descartes_model_))
        ROS_ERROR("this pose has some serious problem");
      
      std::string str = t_place.id_ + ": RRT plan failed!";
      res.success = false;
      reset();
      return true;
    }
    nh.deleteParam("/preload_path");
    
    t_place.setPlannedTrajectoryToParam();
    
    for (auto j:t_pick.getTargetJoints())
      res.pick_joints.push_back(j);
    
    for (auto j:t_place.getTargetJoints())
      res.place_joints.push_back(j);
    
    res.manipulability.push_back ( manipulability(t_pick .getTargetJoints()) );
    res.manipulability.push_back ( manipulability(t_place.getTargetJoints()) );
    res.success = true;
  }

  bool PickPlaceManipulability::manipulabilityCallback(manipulability_pkg_msgs::pickPlaceManipulabilityRequest& req, manipulability_pkg_msgs::pickPlaceManipulabilityResponse& res)
  {
    if (!configAndCheck(req))
    {
      res.success = false;
      reset();
      return true;
    }
    
    double mu_pick ;
    double mu_place;
    
    {
      ROS_DEBUG_STREAM("id: "<<req.pick_name);
      Trajectory t(req.pick_name, nh, joint_home_.size()) ;
      if (!getTrajectory(t, req)) 
      {
        ROS_ERROR ( "error in getting trajectory" );
        res.success = false;
        reset();
        return true;
      }
      
      move_group_->execute( generateTwoPointsTraj(move_group_->getCurrentJointValues(), t.getTargetJoints()));
      
      geometry_msgs::Pose p = t.getTargetPose();
      ROS_DEBUG_STREAM("pick pose target");
      ROS_DEBUG_STREAM(p);
//       std::cin.get();
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      planning_scene_->checkSelfCollision(collision_request, collision_result);
      
      if (collision_result.collision )
      {
        ROS_ERROR_STREAM("self collisoin found in pick pose");
        res.success = false;
        reset();
        return true;
      }
      
      ROS_DEBUG_STREAM("Test 1: Current state is "
                      << (collision_result.collision ? "in" : "not in")
                      << " self collision");
      
      for (auto r : t.getTargetJoints())
        res.pick_joints.push_back(r);
      
      mu_pick = manipulability(t.getTargetJoints());
    }
    {
      ROS_DEBUG_STREAM("id: "<<req.place_name);
      Trajectory t(req.place_name, nh, joint_home_.size()) ;
      if (!getTrajectory(t, req)) 
      {
        ROS_ERROR ( "error in getting trajectory" );
        res.success = false;
        reset();
        return true;
      }
      
      std::string object = object_name_   ; object +="_0";
      std::string environment = environment_name_; environment +="_0";
      
      object_loader_msgs::AttachObject ob;
      ob.request.obj_id = object;
      ob.request.link_name = GRIP_ENDEFFECTOR;
      
      ros::Duration(0.1).sleep();
      att_obj_.call(ob);
      
      move_group_->execute( generateTwoPointsTraj(move_group_->getCurrentJointValues(), t.getTargetJoints()));
      
      geometry_msgs::Pose p = t.getTargetPose();
      ROS_DEBUG_STREAM("place pose target");
      ROS_DEBUG_STREAM(p);
//       std::cin.get();
      collision_detection::CollisionRequest collision_request;
      collision_detection::CollisionResult collision_result;
      planning_scene_->checkSelfCollision(collision_request, collision_result);
      
      if(collision_result.collision )
      {
        ROS_ERROR_STREAM("self collision found in place pose");
        res.success = false;
        reset();
        return true;
      }
      
      checkRobotCollisions(object,environment);
      
      for (auto r : t.getTargetJoints())
        res.place_joints.push_back(r);
      
      mu_place = manipulability(t.getTargetJoints());
    }
    
    if(mu_pick < 0 || mu_place < 0)
    {
      ROS_FATAL_STREAM("manipulability index negative! mu_pick: "<< mu_pick <<", mu_place: " << mu_place << ". check joint limits");
      res.success = false;
      reset();
      return true;
    }
    
    res.manipulability.push_back ( mu_pick );
    res.manipulability.push_back ( mu_place );
    
    res.success = true;
    
    return true;
  }

