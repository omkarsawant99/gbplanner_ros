#include "gbplanner_ui.h"

#include <cmath>
// pci_initialization_trigger
namespace gbplanner_ui {

gbplanner_panel::gbplanner_panel(QWidget* parent) : rviz::Panel(parent) {
  planner_client_start_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/automatic_planning");
  planner_client_start_planner_single = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/single_planning");   
  planner_client_stop_planner = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/stop");
  planner_client_homing = nh.serviceClient<std_srvs::Trigger>(
      "/planner_control_interface/std_srvs/homing_trigger");
  planner_client_init_motion =
      nh.serviceClient<planner_msgs::pci_initialization>(
          "pci_initialization_trigger");
  ros::NodeHandle private_nh("~");
  std::string waypoint_service;
  std::string waypoint_button_label;
  private_nh.param<std::string>("waypoint_service", waypoint_service,
                                "/planner_control_interface/std_srvs/go_to_waypoint");
  private_nh.param<std::string>("waypoint_button_label", waypoint_button_label,
                                "Plan to Waypoint");
  private_nh.param<bool>("inspection_waypoint_sequence", inspection_waypoint_sequence, false);
  private_nh.param<double>("inspection_hover_s", inspection_hover_s, 3.0);
  private_nh.param<double>("inspection_ascent_timeout_s", inspection_ascent_timeout_s, 30.0);
  std::string altitude_command_topic;
  std::string altitude_odometry_topic;
  private_nh.param<std::string>("altitude_command_topic", altitude_command_topic,
                                "command/trajectory");
  private_nh.param<std::string>("altitude_odometry_topic", altitude_odometry_topic,
                                "odometry");
  private_nh.param<double>("altitude_setpoint_z", altitude_setpoint_z, 1.0);
  private_nh.param<std::string>("altitude_frame_id", altitude_frame_id, "world");
  altitude_setpoint_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(altitude_command_topic, 1);
  odometry_sub = nh.subscribe(altitude_odometry_topic, 1,
                              &gbplanner_panel::odometry_callback, this);
  planner_client_plan_to_waypoint =
      nh.serviceClient<std_srvs::Trigger>(waypoint_service);
  planner_client_global_planner =
      nh.serviceClient<planner_msgs::pci_global>("pci_global");
  change_operation_mode_client = nh.serviceClient<std_srvs::SetBool>(
        "gbplanner/switch_operation_mode");

  QVBoxLayout* v_box_layout = new QVBoxLayout;

  button_start_planner = new QPushButton;
  button_start_planner_single = new QPushButton;
  button_stop_planner = new QPushButton;
  button_homing = new QPushButton;
  button_init_motion = new QPushButton;
  button_altitude_setpoint = inspection_waypoint_sequence ? nullptr : new QPushButton;
  button_plan_to_waypoint = new QPushButton;
  button_global_planner = new QPushButton;
  button_change_operation_mode = new QPushButton;
  inspection_preflight_timer = new QTimer(this);
  inspection_preflight_timer->setInterval(200);
  connect(inspection_preflight_timer, SIGNAL(timeout()), this,
          SLOT(on_inspection_preflight_tick()));

  button_start_planner->setText("Start Planner");
  button_start_planner_single->setText("Start Single Planner");
  button_stop_planner->setText("Stop Planner");
  button_homing->setText("Go Home");
  button_init_motion->setText("Initialization");
  if (button_altitude_setpoint) {
    button_altitude_setpoint->setText(
        QString("Move to z = %1 m").arg(altitude_setpoint_z, 0, 'f', 1));
  }
  button_plan_to_waypoint->setText(QString::fromStdString(waypoint_button_label));
  button_global_planner->setText("Run Global");
  button_change_operation_mode->setText("Operation Mode (EXP)");

  v_box_layout->addWidget(button_start_planner);
  v_box_layout->addWidget(button_start_planner_single);
  v_box_layout->addWidget(button_stop_planner);
  v_box_layout->addWidget(button_homing);
  v_box_layout->addWidget(button_init_motion);
  if (button_altitude_setpoint) v_box_layout->addWidget(button_altitude_setpoint);
  v_box_layout->addWidget(button_plan_to_waypoint);
  v_box_layout->addWidget(button_change_operation_mode);

  QVBoxLayout* global_vbox_layout = new QVBoxLayout;
  QHBoxLayout* global_hbox_layout = new QHBoxLayout;

  QLabel* text_label_ptr = new QLabel("Frontier ID:");

  global_id_line_edit = new QLineEdit();

  global_hbox_layout->addWidget(text_label_ptr);
  global_hbox_layout->addWidget(global_id_line_edit);
  global_hbox_layout->addWidget(button_global_planner);
  global_vbox_layout->addLayout(global_hbox_layout);
  v_box_layout->addLayout(global_vbox_layout);

  setLayout(v_box_layout);

  connect(button_start_planner, SIGNAL(clicked()), this,
          SLOT(on_start_planner_click()));
  connect(button_start_planner_single, SIGNAL(clicked()), this,
          SLOT(on_start_planner_single_click()));
  connect(button_stop_planner, SIGNAL(clicked()), this,
          SLOT(on_stop_planner_click()));
  connect(button_homing, SIGNAL(clicked()), this, SLOT(on_homing_click()));
  connect(button_init_motion, SIGNAL(clicked()), this,
          SLOT(on_init_motion_click()));
  if (button_altitude_setpoint) {
    connect(button_altitude_setpoint, SIGNAL(clicked()), this,
            SLOT(on_altitude_setpoint_click()));
  }
  connect(button_plan_to_waypoint, SIGNAL(clicked()), this,
          SLOT(on_plan_to_waypoint_click()));
  connect(button_global_planner, SIGNAL(clicked()), this,
          SLOT(on_global_planner_click()));
  connect(button_change_operation_mode, SIGNAL(clicked()), this, SLOT(on_change_operation_mode_click()));
}

void gbplanner_panel::on_start_planner_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_start_planner.getService().c_str());
  }
}

void gbplanner_panel::on_start_planner_single_click() {
  std_srvs::Trigger srv;
  if (!planner_client_start_planner_single.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_start_planner_single.getService().c_str());
  }
}

void gbplanner_panel::on_stop_planner_click() {
  std_srvs::Trigger srv;
  inspection_preflight_timer->stop();
  inspection_preflight_active = false;
  inspection_planner_started = false;
  if (!planner_client_stop_planner.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_stop_planner.getService().c_str());
  }
}

void gbplanner_panel::on_homing_click() {
  std_srvs::Trigger srv;
  if (!planner_client_homing.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_homing.getService().c_str());
  }
}

void gbplanner_panel::on_init_motion_click() {
  planner_msgs::pci_initialization srv;
  if (!planner_client_init_motion.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_init_motion.getService().c_str());
  }
}

void gbplanner_panel::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(odometry_mutex);
  latest_odometry = *msg;
  odometry_received = true;
  last_odometry_wall_time = ros::WallTime::now();
}

trajectory_msgs::MultiDOFJointTrajectory gbplanner_panel::make_altitude_command(
    const nav_msgs::Odometry& odometry) const {
  trajectory_msgs::MultiDOFJointTrajectory trajectory;
  trajectory.header.stamp = ros::Time::now();
  trajectory.header.frame_id = altitude_frame_id;
  trajectory.joint_names.push_back("base_link");

  trajectory_msgs::MultiDOFJointTrajectoryPoint point;
  geometry_msgs::Transform transform;
  transform.translation.x = odometry.pose.pose.position.x;
  transform.translation.y = odometry.pose.pose.position.y;
  transform.translation.z = altitude_setpoint_z;
  const geometry_msgs::Quaternion& current_q = odometry.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (current_q.w * current_q.z +
                                       current_q.x * current_q.y),
                                1.0 - 2.0 * (current_q.y * current_q.y +
                                             current_q.z * current_q.z));
  transform.rotation.z = std::sin(0.5 * yaw);
  transform.rotation.w = std::cos(0.5 * yaw);
  point.transforms.push_back(transform);
  point.time_from_start = ros::Duration(0.0);
  trajectory.points.push_back(point);

  return trajectory;
}

void gbplanner_panel::on_altitude_setpoint_click() {
  nav_msgs::Odometry odometry;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex);
    if (!odometry_received) {
      ROS_ERROR("[GBPLANNER-UI] Cannot send altitude setpoint: no odometry received");
      return;
    }
    odometry = latest_odometry;
  }
  trajectory_msgs::MultiDOFJointTrajectory trajectory = make_altitude_command(odometry);

  altitude_setpoint_pub.publish(trajectory);
  ROS_INFO("[GBPLANNER-UI] Sent position setpoint [%.2f, %.2f, %.2f] in %s",
           trajectory.points[0].transforms[0].translation.x,
           trajectory.points[0].transforms[0].translation.y,
           trajectory.points[0].transforms[0].translation.z,
           altitude_frame_id.c_str());
}

void gbplanner_panel::on_plan_to_waypoint_click() {
  if (inspection_waypoint_sequence) {
    if (inspection_preflight_active || inspection_planner_started) return;
    nav_msgs::Odometry odometry;
    {
      std::lock_guard<std::mutex> lock(odometry_mutex);
      if (!odometry_received ||
          (ros::WallTime::now() - last_odometry_wall_time).toSec() > 2.0) {
        ROS_ERROR("[GBPLANNER-UI] Cannot start inspection: odometry is unavailable or stale");
        return;
      }
      odometry = latest_odometry;
    }
    if (odometry.header.frame_id != altitude_frame_id) {
      ROS_ERROR("[GBPLANNER-UI] Odometry frame %s differs from altitude frame %s",
                odometry.header.frame_id.c_str(), altitude_frame_id.c_str());
      return;
    }
    inspection_altitude_command = make_altitude_command(odometry);
    inspection_started_at = ros::WallTime::now();
    inspection_stable_since = ros::WallTime();
    inspection_preflight_active = true;
    inspection_preflight_timer->start();
    on_inspection_preflight_tick();
    ROS_INFO("[GBPLANNER-UI] start_inspection_waypoint: moving to %.2f m",
             altitude_setpoint_z);
    return;
  }
  std_srvs::Trigger srv;
  if (!planner_client_plan_to_waypoint.call(srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_plan_to_waypoint.getService().c_str());
  } else if (!srv.response.success) {
    ROS_ERROR("[GBPLANNER-UI] %s: %s",
              planner_client_plan_to_waypoint.getService().c_str(),
              srv.response.message.c_str());
  }
}

void gbplanner_panel::on_inspection_preflight_tick() {
  if (!inspection_preflight_active) return;
  const ros::WallTime now = ros::WallTime::now();
  if ((now - inspection_started_at).toSec() > inspection_ascent_timeout_s) {
    ROS_ERROR("[GBPLANNER-UI] Inspection start aborted: altitude/hover timeout");
    inspection_preflight_timer->stop();
    inspection_preflight_active = false;
    return;
  }
  inspection_altitude_command.header.stamp = ros::Time::now();
  altitude_setpoint_pub.publish(inspection_altitude_command);

  nav_msgs::Odometry odometry;
  ros::WallTime received_at;
  {
    std::lock_guard<std::mutex> lock(odometry_mutex);
    odometry = latest_odometry;
    received_at = last_odometry_wall_time;
  }
  const auto& goal = inspection_altitude_command.points[0].transforms[0].translation;
  const auto& p = odometry.pose.pose.position;
  const auto& v = odometry.twist.twist.linear;
  const bool stable = (now - received_at).toSec() <= 1.0 &&
      std::abs(p.z - goal.z) <= 0.12 &&
      std::hypot(p.x - goal.x, p.y - goal.y) <= 0.25 &&
      std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z) <= 0.12;
  if (!stable) {
    inspection_stable_since = ros::WallTime();
    return;
  }
  if (inspection_stable_since.isZero()) inspection_stable_since = now;
  if ((now - inspection_stable_since).toSec() < inspection_hover_s) return;

  inspection_preflight_timer->stop();
  inspection_preflight_active = false;
  std_srvs::Trigger srv;
  if (!planner_client_start_planner.call(srv) || !srv.response.success) {
    ROS_ERROR("[GBPLANNER-UI] Failed to start waypoint inspection planner");
    return;
  }
  inspection_planner_started = true;
  ROS_INFO("[GBPLANNER-UI] Hovered for %.1f s; waypoint inspection planner started",
           inspection_hover_s);
}

void gbplanner_panel::on_global_planner_click() {
  // retrieve ID as a string
  std::string in_string = global_id_line_edit->text().toStdString();
  // global_id_line_edit->clear();
  int id = -1;
  if (in_string.empty())
    id = 0;
  else {
    // try to convert to an integer
    try {
      id = std::stoi(in_string);
    } catch (const std::out_of_range& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    } catch (const std::invalid_argument& exc) {
      ROS_ERROR("[GBPLANNER UI] - Invalid ID: %s", in_string.c_str());
      return;
    }
  }
  // check bounds on integer
  if (id < 0) {
    ROS_ERROR("[GBPLANNER UI] - In valid ID, must be non-negative");
    return;
  }
  // we got an ID!!!!!!!!!
  ROS_INFO("Global Planner found ID : %i", id);

  planner_msgs::pci_global plan_srv;
  plan_srv.request.id = id;
  if (!planner_client_global_planner.call(plan_srv)) {
    ROS_ERROR("[GBPLANNER-UI] Service call failed: %s",
              planner_client_global_planner.getService().c_str());
  }
}

void gbplanner_panel::on_change_operation_mode_click()
{
  std_srvs::SetBool srv;
  waypoint_nav_mode = !waypoint_nav_mode;
  srv.request.data = waypoint_nav_mode;
  if (!change_operation_mode_client.call(srv))
  {
    ROS_ERROR("[GBPLANNER UI] Service call failed: %s",
              change_operation_mode_client.getService().c_str());
  }
  if (waypoint_nav_mode)
  {
    button_change_operation_mode->setText("Operation mode (WP)");
  }
  else
  {
    button_change_operation_mode->setText("Operation mode (EXP)");
  }
}

void gbplanner_panel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}
void gbplanner_panel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

}  // namespace gbplanner_ui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gbplanner_ui::gbplanner_panel, rviz::Panel)
