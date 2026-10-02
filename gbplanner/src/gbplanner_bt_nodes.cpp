#include "gbplanner/gbplanner_bt_nodes.h"

/***************** LocalExploration **********************/
BT::NodeStatus LocalExploration::onStart()
{
  std::cout << "[Local Exploration] Triggered." << std::endl;
  Rrg::LocalPlannerStatus status = gbplanner_->getExplorationPath();
  if(status == Rrg::LocalPlannerStatus::L_EXHAUSTED)
  {
    gbplanner_->bt_states_.local_exp_exhausted = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    gbplanner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalExploration::onHalted()
{
  std::cout << "[Local Exploration] Halted" << std::endl;
}
/*******************************************************/

/***************** LocalExpExhaustedCheck **************/
BT::NodeStatus LocalExpExhaustedCheck::tick()
{
  if(gbplanner_->bt_states_.local_exp_exhausted)
  {
    ROS_WARN("Local Exp Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}
/*******************************************************/

/***************** LocalExpExhaustedReset **************/
BT::NodeStatus LocalExpExhaustedReset::tick()
{
  gbplanner_->bt_states_.local_exp_exhausted = false;
  gbplanner_->clearResPath();

  return BT::NodeStatus::SUCCESS;
}
/*******************************************************/

/***************** SwitchToLocalNavigation **************/
BT::NodeStatus SwitchToLocalNavigation::tick()
{
  if(gbplanner_->bt_states_.operation_mode)
  {
    ROS_WARN("Using Local Navigation Mode");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** LocalNavigation **********************/
BT::NodeStatus LocalNavigation::onStart()
{
  std::cout << "[Local Navigation] Triggered." << std::endl;
  Rrg::LocalPlannerStatus status = gbplanner_->getLocalNavigationPath();
  if(status == Rrg::LocalPlannerStatus::L_EXHAUSTED)
  {
    gbplanner_->bt_states_.local_navigation_complete = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_STUCK)
  {
    gbplanner_->bt_states_.local_navigation_stuck = true;
  }
  else if(status == Rrg::LocalPlannerStatus::L_TIME_LIMIT_REACHED)
  {
    gbplanner_->bt_states_.homing_triggered = true;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus LocalNavigation::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void LocalNavigation::onHalted()
{
  std::cout << "[Local Navigation] Halted" << std::endl;
}
/*******************************************************/

/***************** LocalNavigationExhaustedCheck **************/
BT::NodeStatus LocalNavigationExhaustedCheck::tick()
{
  std::cout << "[LocalNavigationExhaustedCheck] Triggered." << std::endl;
  if(gbplanner_->bt_states_.local_navigation_complete)
  {
    ROS_WARN("Local Navigation Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::FAILURE;
}
/*******************************************************/

/***************** LocalNavigationExhaustedReset **************/
BT::NodeStatus LocalNavigationExhaustedReset::tick()
{
  gbplanner_->bt_states_.local_navigation_complete = false;
  gbplanner_->clearResPath();

  return BT::NodeStatus::SUCCESS;
}
/*******************************************************/


/***************** GlobalExploration **********************/
BT::NodeStatus GlobalExploration::onStart()
{
  std::cout << "[Global Exploration] Triggered." << std::endl;
  gbplanner_->bt_states_.local_exp_exhausted = false;
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_exp_count_, 2);  // TODO: Set the max bound number through param

  Rrg::GlobalPlannerStatus status = gbplanner_->getGlobalExplorationPath();
  if(status == Rrg::GlobalPlannerStatus::G_ERR)
  {
    ++failed_exp_count_;
    if(failed_exp_count_ > max_global_planner_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else if(status == Rrg::GlobalPlannerStatus::G_HOMING)
  {
    ROS_WARN("[GE]: Homing Triggered");
    gbplanner_->bt_states_.homing_triggered = true;
    failed_exp_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    failed_exp_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus GlobalExploration::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void GlobalExploration::onHalted()
{
  std::cout << "[Global Exploration] Halted" << std::endl;
}
/*******************************************************/

/***************** GlobalExpExhaustedCheck **************/
BT::NodeStatus GlobalExpExhaustedCheck::tick()
{
  ROS_WARN("[GlobalExpExhaustedCheck]");
  // if(gbplanner_->bt_states_.global_exp_exhausted)
  if(gbplanner_->checkGlobalExplorationStatus())
  {
    ROS_WARN("Global Exp Exhausted");
    return BT::NodeStatus::SUCCESS;
  }
  gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** CalculateGlobalPath **************/
BT::NodeStatus CalculateGlobalPath::tick()
{
  ROS_WARN("[CalculateGlobalPath node triggered]");
  gbplanner_->bt_states_.local_exp_exhausted = false;
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_global_planner_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->calculateGlobalPath();
  if(!success)
  {
    ROS_WARN("Global planner failed");
    ++failed_global_planner_count_;
    if(failed_global_planner_count_ > max_global_planner_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_WARN("Calculated Global Path Successfully");
    failed_global_planner_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/

/***************** UpdateGlobalGoal **************/
BT::NodeStatus UpdateGlobalGoal::tick()
{
  ROS_WARN("[UpdateGlobalGoal node triggered]");

  bool success = gbplanner_->updateGlobalGoal();
  if(!success)
  {
    ROS_WARN("Global Goal Update: Completed");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_WARN("Global Goal Update: Continuing");
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** Inspection **********************/
BT::NodeStatus Inspection::onStart()
{
  ROS_INFO("[Inspection] Triggered.");
  // std::cout << "[Global Exploration] Triggered." << std::endl;
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_inspection_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->getInspectionPath();
  if(!success)
  {
    ++failed_inspection_count_;
    if(failed_inspection_count_ > max_inspection_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    failed_inspection_count_ = 0;
    return BT::NodeStatus::RUNNING;
  }
}

BT::NodeStatus Inspection::onRunning()
{
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void Inspection::onHalted()
{
  std::cout << "[Inspection] Halted" << std::endl;
}
/*******************************************************/


BT::NodeStatus BuildInspectionGraph::onStart()
{
  ROS_INFO("[BuildInspectionGraph] Preparing global graph from inspection map");
  if (gbplanner_->prepareWaypointGraph()) return BT::NodeStatus::SUCCESS;
  next_retry_ = ros::WallTime::now() + ros::WallDuration(2.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BuildInspectionGraph::onRunning()
{
  if (ros::WallTime::now() < next_retry_) return BT::NodeStatus::RUNNING;
  if (gbplanner_->prepareWaypointGraph()) return BT::NodeStatus::SUCCESS;
  next_retry_ = ros::WallTime::now() + ros::WallDuration(2.0);
  ROS_WARN_THROTTLE(5.0, "Inspection graph not ready; retrying");
  return BT::NodeStatus::RUNNING;
}

/***************** CompartmentTransition **********************/
BT::NodeStatus CompartmentTransition::onStart()
{
  ROS_INFO("[CompartmentTransition] Triggered.");
  // std::cout << "[Global Exploration] Triggered." << std::endl;
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_compartment_transition_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->getCompartmentTransitionPath();
  ROS_WARN("Compartment transition returned %d", success);
  if(!success)
  {
    ++failed_compartment_transition_count_;
    if(failed_compartment_transition_count_ > max_compartment_transition_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    failed_compartment_transition_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

BT::NodeStatus CompartmentTransition::onRunning()
{
  ROS_INFO("[CompartmentTransition] Running.");
  return BT::NodeStatus::SUCCESS;  // Does nothing for now
}

void CompartmentTransition::onHalted()
{
  std::cout << "[CompartmentTransition] Halted" << std::endl;
}
/*******************************************************/


/***************** Homing **************/
BT::NodeStatus Homing::tick()
{
  ROS_WARN("[Homing node triggered]");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->getHomingPath();
  if(!success)
  {
    ROS_WARN("Homing failed");
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_WARN("Homing Succeeded");
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** HomingCheck **************/
BT::NodeStatus HomingCheck::tick()
{
  if(gbplanner_->bt_states_.homing_required)
  {
    ROS_WARN("Homing needed 0");
    return BT::NodeStatus::SUCCESS;
  }
  
  bool homing_reqd = gbplanner_->homingRequired();
  if(homing_reqd)
  {
    ROS_WARN("Homing needed 1");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_WARN("Homing NOT needed");
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/


/***************** CalculateHomingPath **************/
BT::NodeStatus CalculateHomingPath::tick()
{
  ROS_WARN("[CalculateHomingPath node triggered]");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_homing_count_, 2);  // TODO: Set the max bound number through param

  bool success = gbplanner_->calculateHomingPath();
  if(!success)
  {
    ROS_WARN("Homing failed");
    ++failed_homing_count_;
    if(failed_homing_count_ > max_homing_tries_)
    {
      gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_WARN("Calculated Homing Path Successfully");
    failed_homing_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}
/*******************************************************/


/***************** UpdateHomingGoal **************/
BT::NodeStatus UpdateHomingGoal::tick()
{
  ROS_WARN("[UpdateHomingGoal node triggered]");

  bool success = gbplanner_->updateHomingGoal();
  if(!success)
  {
    ROS_WARN("Homing Goal Update: Completed");
    return BT::NodeStatus::FAILURE;
  }
  else
  {
    ROS_WARN("Homing Goal Update: Continuing");
    return BT::NodeStatus::SUCCESS;
  }
}




/***************** OPENINGPhase1 **********************/
BT::NodeStatus OPENINGPhase1::onStart()
{
  ROS_INFO("[OPENINGPhase1] Triggered.");
  gbplanner_->in_srv_req_.bound_mode = std::min(failed_opening_phase1_count_, 2);  // TODO: Set the max bound number through param

  OpeningTraversalMode mode = OpeningTraversalMode::kGoingTo;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  // The detector needs several clouds before an opening becomes stable. Keep
  // this service request alive long enough to receive its first detections;
  // otherwise the Idle branch stops PCI after a single early empty result.
  double detection_wait_s = 15.0;
  ros::NodeHandle("~").param("opening_detection_wait_s", detection_wait_s, detection_wait_s);
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(std::max(0.0, detection_wait_s));
  while (status == OpeningTraversalStatus::NO_OPENINGS && ros::ok() &&
         ros::WallTime::now() < deadline)
  {
    ros::WallDuration(0.25).sleep();
    ros::spinOnce();
    gbplanner_->getOpeningTraversalPath(mode, status);
  }
  if(status == OpeningTraversalStatus::CANT_CONNECT)
  {
    ++failed_opening_phase1_count_;
    return BT::NodeStatus::FAILURE;
  }
  else if(status == OpeningTraversalStatus::OK)
  {
    failed_opening_phase1_count_ = 0;
    gbplanner_->bt_states_.opening_phase1_failed = false;
    return BT::NodeStatus::RUNNING;
  }
  else if(status == OpeningTraversalStatus::NO_OPENINGS)
  {
    gbplanner_->bt_states_.opening_phase1_failed = true;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhase1::onRunning()
{
  OpeningTraversalMode mode = OpeningTraversalMode::kPathCheck;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);

  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

void OPENINGPhase1::onHalted()
{
  std::cout << "[OPENINGPhase1] Halted" << std::endl;
}
/*******************************************************/


/***************** OPENINGP1FailCheck **************/
BT::NodeStatus OPENINGP1FailCheck::tick()
{
  if(gbplanner_->bt_states_.opening_phase1_failed)
  {
    ROS_WARN("OPENING Phase1 Failed");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/


/***************** OPENINGPhaseCheck **********************/
BT::NodeStatus OPENINGPhaseCheck::onStart()
{
  ROS_INFO("[OPENINGPhaseCheck] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPathCheck;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhaseCheck::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void OPENINGPhaseCheck::onHalted()
{
  std::cout << "[OPENINGPhaseCheck] Halted" << std::endl;
}
/*******************************************************/


/***************** OPENINGPhase2 **********************/
BT::NodeStatus OPENINGPhase2::onStart()
{
  ROS_INFO("[OPENINGPhase2] Triggered.");

  OpeningTraversalMode mode = OpeningTraversalMode::kPassingThrough;
  OpeningTraversalStatus status;

  gbplanner_->getOpeningTraversalPath(mode, status);
  if(status == OpeningTraversalStatus::OK)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus OPENINGPhase2::onRunning()
{
  return BT::NodeStatus::SUCCESS;
}

void OPENINGPhase2::onHalted()
{
  std::cout << "[OPENINGPhase2] Halted" << std::endl;
}
/*******************************************************/


/***************** SetNextCompartment **************/
BT::NodeStatus SetNextCompartment::tick()
{
  if(gbplanner_->transitionCompartment())
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/

BT::NodeStatus BeginWaypointCompartment::tick()
{
  return gbplanner_->beginWaypointCompartment() ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
}

void InspectionWaypoint::resetMeasurement()
{
  measurement_requested_ = false;
  measurement_started_ = false;
  measurement_done_ = false;
  next_measurement_retry_ = ros::WallTime();
}

bool InspectionWaypoint::measurementComplete(
    std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  res.success = measurement_started_;
  if (!res.success) {
    res.message = "No active NDT measurement";
    return true;
  }
  measurement_done_ = true;
  res.message = "Completion recorded; navigation may continue on the next planner tick";
  ROS_INFO("NDT measurement completed for target %d", target_index_);
  return true;
}

BT::NodeStatus InspectionWaypoint::step()
{
  gbplanner_->out_srv_res_.path.clear();
  gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kAutoCustomPath;
  const int compartment = gbplanner_->currentWaypointCompartment();
  if (compartment < 0 || !gbplanner_->hasWaypointOdometry()) {
    ROS_WARN_THROTTLE(5.0, "Waiting for waypoint compartment and odometry");
    return BT::NodeStatus::RUNNING;
  }
  if (compartment != compartment_index_) {
    resetMeasurement();
    compartment_index_ = compartment;
    target_index_ = -1;
    target_resolved_ = false;
    revision_.clear();
    next_path_retry_ = ros::WallTime();
    reached_since_ = ros::Time();
  }

  const bool within_target = target_resolved_ && target_index_ >= 0 &&
      gbplanner_->waypointTargetReached(target_, reach_radius_m_);
  const ros::Time now = ros::Time::now();
  if (!within_target) reached_since_ = ros::Time();
  else if (reached_since_.isZero() || now < reached_since_) reached_since_ = now;
  const bool reached = measurement_enabled_ ? measurement_done_ :
      (within_target && (now - reached_since_).toSec() >= reach_hold_s_);
  planner_msgs::GetInspectionTarget service;
  bool has_entry_wall = false, has_exit_wall = false;
  if (!gbplanner_->waypointCompartmentWalls(
          service.request.entry_wall, has_entry_wall,
          service.request.exit_wall, has_exit_wall,
          service.request.forward)) {
    return BT::NodeStatus::RUNNING;
  }
  service.request.has_entry_wall = has_entry_wall;
  service.request.has_exit_wall = has_exit_wall;
  service.request.compartment_index = compartment;
  service.request.completed_target_index = reached ? target_index_ : -1;
  service.request.completed_revision = reached ? revision_ : "";
  if (!target_client_.waitForExistence(ros::Duration(0.5)) || !target_client_.call(service)) {
    ROS_WARN_THROTTLE(5.0, "Waiting for AutoAssess compartment target service");
    return BT::NodeStatus::RUNNING;
  }
  const auto& result = service.response;
  if (!result.ready) {
    ROS_WARN_THROTTLE(5.0, "Inspection targets unavailable: %s", result.message.c_str());
    return BT::NodeStatus::RUNNING;
  }
  if (target_index_ >= 0 && revision_ != result.revision) {
    resetMeasurement();
    ROS_WARN("AutoAssess target plan changed; discarding previous waypoint");
    target_index_ = -1;
    target_resolved_ = false;
    next_path_retry_ = ros::WallTime();
    reached_since_ = ros::Time();
  }
  if (reached && result.revision == revision_) {
    ROS_INFO("Inspection target %d complete (%s)", target_index_,
             measurement_enabled_ ? "NDT completion received" : "arrival hold finished");
    resetMeasurement();
    target_index_ = -1;
    target_resolved_ = false;
    reached_since_ = ros::Time();
  }
  if (!result.has_target) {
    resetMeasurement();
    ROS_INFO("No remaining AutoAssess targets in compartment %d", compartment);
    return BT::NodeStatus::FAILURE;
  }
  if (target_index_ != result.target_index) {
    resetMeasurement();
    target_index_ = result.target_index;
    requested_target_ = result.target;
    target_ = result.target;
    target_resolved_ = false;
    revision_ = result.revision;
    next_path_retry_ = ros::WallTime();
    reached_since_ = ros::Time();
    ROS_INFO("Planning to AutoAssess target %d in compartment %d",
             target_index_, compartment);
  }
  if (measurement_enabled_ && measurement_started_) {
    // The position controller retains its endpoint command. Never send another
    // navigation path while measurement owns this workflow stage.
    return BT::NodeStatus::RUNNING;
  }
  if (target_resolved_ && (gbplanner_->waypointTargetReached(target_, reach_radius_m_) ||
                          (measurement_enabled_ && measurement_requested_))) {
    if (measurement_enabled_) {
      if (ros::WallTime::now() < next_measurement_retry_) return BT::NodeStatus::RUNNING;
      measurement_requested_ = true;
      std_srvs::Trigger request;
      next_measurement_retry_ = ros::WallTime::now() + ros::WallDuration(2.0);
      if (!measurement_client_.exists() || !measurement_client_.call(request) ||
          !request.response.success) {
        ROS_WARN_THROTTLE(5.0, "Waiting for NDT start acceptance; holding target %d", target_index_);
        return BT::NodeStatus::RUNNING;
      }
      measurement_started_ = true;
      ROS_INFO("NDT measurement started for target %d; holding until completion", target_index_);
      return BT::NodeStatus::RUNNING;
    }
    // Keep the controller's endpoint setpoint while the arrival dwell elapses.
    return BT::NodeStatus::RUNNING;
  }
  if (ros::WallTime::now() < next_path_retry_) {
    return BT::NodeStatus::RUNNING;
  }
  if (!gbplanner_->planWaypointTarget(requested_target_, target_)) {
    ROS_WARN_THROTTLE(5.0, "No graph path to AutoAssess target %d; retaining it", target_index_);
    next_path_retry_ = ros::WallTime::now() + ros::WallDuration(2.0);
    return BT::NodeStatus::RUNNING;
  }
  target_resolved_ = true;
  reached_since_ = ros::Time();
  if (gbplanner_->waypointTargetReached(target_, reach_radius_m_)) {
    gbplanner_->out_srv_res_.path.clear();
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::SUCCESS;
}

/***************** AllCompartmentsInspectedCheck **************/
BT::NodeStatus AllCompartmentsInspectedCheck::tick()
{
  if(gbplanner_->allCompartmentsInspected())
  {
    ROS_WARN("[AllCompartmentsInspectedCheck]");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
/*******************************************************/


/***************** Idle **************/
BT::NodeStatus Idle::tick()
{
  gbplanner_->out_srv_res_.status = planner_msgs::planner_srv::Response::kManualCustomPath;  
  gbplanner_->out_srv_res_.path.clear();
  return BT::NodeStatus::FAILURE;
}
/*******************************************************/
