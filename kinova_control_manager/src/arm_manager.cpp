#include <mutex>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <mrs_lib/ParamLoader.h>
#include <mrs_msgs/Vec4.h>

#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_arm.h>

#include <std_srvs/Trigger.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/PoseVelocity.h>

#include <kinova_control_manager/ArmStatus.h>
#include <kinova_control_manager/EndEffectorPose.h>
#include <kinova_msgs/ArmPoseActionGoal.h>

#define DOF 6

namespace kinova_control_manager
{

typedef enum
{
  MOVING,
  IDLE,
  HOMING,
} MotionStatus_t;

/* struct Pose3d //{ */
struct Pose3d
{
  Eigen::Vector3d    pos;  // [m] absolute position in robot base frame
  Eigen::Quaterniond rot;  // absolute rotation in robot base frame

  bool operator!=(const Pose3d &p) const {
    return pos != p.pos || rot.coeffs() != p.rot.coeffs();
  }
  bool operator==(const Pose3d &p) const {
    return pos == p.pos && rot.coeffs() == p.rot.coeffs();
  }

  Pose3d operator+(const Pose3d &p) const {
    Pose3d result;
    result.pos = pos + p.pos;
    result.rot = rot * p.rot;
    return result;
  }

  Pose3d operator-(const Pose3d &p) const {
    Pose3d result;
    result.pos = pos - p.pos;
    result.rot = rot * p.rot.inverse();
    result.rot.normalize();
    return result;
  }
};
//}

/* class ArmManager //{ */
class ArmManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized       = false;
  bool            getting_joint_angles = false;
  bool            getting_effector_pos = false;
  std::string     arm_type;
  std::mutex      arm_state_mutex;

  // continuous status publishing
  ros::Timer status_timer;
  int        status_timer_rate;
  void       statusTimer(const ros::TimerEvent &evt);

  // arm motion checker
  ros::Timer motion_check_timer;
  int        motion_check_timer_rate;
  void       motionCheckTimer(const ros::TimerEvent &evt);

  // goto execution
  void goTo(Pose3d pose);
  void goToRelative(Pose3d pose);
  void goToRelativeGripping(Pose3d pose);

  void setVelocity(Eigen::Vector3d cartesian_velocity);

  // configuration params
  Pose3d             home_pose;
  Pose3d             default_gripping_pose;
  Eigen::Quaterniond gripper_down;
  double             nearby_position_threshold;
  double             nearby_rotation_threshold;
  double             no_move_error_timeout;

  // arm status
  MotionStatus_t status;
  double         joint_angles[DOF];

  Pose3d end_effector_pose;
  Pose3d last_goal;
  double time_since_last_movement;

  double last_joint_angles[DOF];
  void   updateLastJointAngles();

  // geometry & utilities
  bool               nearbyPose(Pose3d p, Pose3d q);
  bool               nearbyAngleDeg(float a, float b);
  bool               nearbyAngleRad(float a, float b);
  Eigen::Vector3d    quaternionToEuler(Eigen::Quaterniond q);
  Eigen::Quaterniond eulerToQuaternion(Eigen::Vector3d euler);
  std::string        statusToString(MotionStatus_t ms);

  // advertised services
  ros::ServiceServer service_server_homing;
  ros::ServiceServer service_server_soft_homing;
  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_relative;
  ros::ServiceServer service_server_prepare_gripping;

  ros::ServiceServer service_server_move_down_until_obstacle;

  // internally called services
  ros::ServiceClient service_client_homing;

  // subscribers
  ros::Subscriber subscriber_joint_angles;
  ros::Subscriber subscriber_end_effector_pose;

  // publishers
  ros::Publisher publisher_arm_status;
  ros::Publisher publisher_end_effector_pose;
  ros::Publisher publisher_cartesian_velocity;

  // service callbacks
  bool callbackHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackSoftHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGoToService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res);
  bool callbackGoToRelativeService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res);
  bool callbackPrepareGrippingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  bool callbackMoveDownUntilObstacleService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


  // subscriber callbacks
  void callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg);
  void callbackEndEffectorPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg);
};
//}

/* onInit (constructor equivalent for nodelets) //{ */
void ArmManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[ArmManager]: Initializing...");

  // helper containers
  std::vector<double> home_pose_raw;
  std::vector<double> gripping_pose_raw;

  // load params
  mrs_lib::ParamLoader param_loader(nh_, "ArmManager");
  param_loader.load_param("arm_type", arm_type);
  param_loader.load_param("status_timer_rate", status_timer_rate);
  param_loader.load_param("motion_check_timer_rate", motion_check_timer_rate);
  param_loader.load_param("no_move_error_timeout", no_move_error_timeout);
  param_loader.load_param("nearby_position_threshold", nearby_position_threshold);
  param_loader.load_param("nearby_rotation_threshold", nearby_rotation_threshold);
  param_loader.load_param("end_effector_home_pose", home_pose_raw);
  param_loader.load_param("default_gripping_pose", gripping_pose_raw);

  if (home_pose_raw.size() != 6) {
    ROS_ERROR("[ArmManager]: Parameter \"end_effector_home\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!", home_pose_raw.size());
    ros::shutdown();
  }

  home_pose.pos = Eigen::Vector3d(home_pose_raw[0], home_pose_raw[1], home_pose_raw[2]);
  home_pose.rot = eulerToQuaternion(Eigen::Vector3d(home_pose_raw[3], home_pose_raw[4], home_pose_raw[5]));

  if (gripping_pose_raw.size() != 6) {
    ROS_ERROR("[ArmManager]: Parameter \"default_gripping_pose\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!", home_pose_raw.size());
    ros::shutdown();
  }

  default_gripping_pose.pos = Eigen::Vector3d(gripping_pose_raw[0], gripping_pose_raw[1], gripping_pose_raw[2]);
  default_gripping_pose.rot = eulerToQuaternion(Eigen::Vector3d(gripping_pose_raw[3], gripping_pose_raw[4], gripping_pose_raw[5]));

  if (arm_type.size() < 4) {
    ROS_ERROR("[ArmManager]: ARM_TYPE expected to be at least 4 characters!");
    ros::shutdown();
  }

  int dof = arm_type[3] - '0';
  if (dof != DOF) {
    ROS_ERROR("[ArmManager]: Expected 6 DOF, got %d!", dof);
    ros::shutdown();
  }

  // service servers
  service_server_homing           = nh_.advertiseService("home_in", &ArmManager::callbackHomingService, this);
  service_server_soft_homing      = nh_.advertiseService("soft_home_in", &ArmManager::callbackSoftHomingService, this);
  service_server_goto             = nh_.advertiseService("goto_in", &ArmManager::callbackGoToService, this);
  service_server_goto_relative    = nh_.advertiseService("goto_relative_in", &ArmManager::callbackGoToRelativeService, this);
  service_server_prepare_gripping = nh_.advertiseService("prepare_gripping_in", &ArmManager::callbackPrepareGrippingService, this);

  service_server_move_down_until_obstacle = nh_.advertiseService("move_down_in", &ArmManager::callbackMoveDownUntilObstacleService, this);

  // service clients
  service_client_homing = nh_.serviceClient<kinova_msgs::HomeArm>("home_out");

  // subscribers
  subscriber_joint_angles      = nh_.subscribe("joint_angles_in", 1, &ArmManager::callbackJointAnglesTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_end_effector_pose = nh_.subscribe("end_effector_pose_in", 1, &ArmManager::callbackEndEffectorPoseTopic, this, ros::TransportHints().tcpNoDelay());

  // publishers
  publisher_arm_status         = nh_.advertise<kinova_control_manager::ArmStatus>("arm_status_out", 1);
  publisher_end_effector_pose  = nh_.advertise<kinova_msgs::ArmPoseActionGoal>("end_effector_pose_out", 1);
  publisher_cartesian_velocity = nh_.advertise<kinova_msgs::PoseVelocity>("cartesian_velocity_out", 1);

  // timers
  status_timer       = nh_.createTimer(ros::Rate(status_timer_rate), &ArmManager::statusTimer, this);
  motion_check_timer = nh_.createTimer(ros::Rate(motion_check_timer_rate), &ArmManager::motionCheckTimer, this);

  status = MotionStatus_t::IDLE;

  for (int i = 0; i < DOF; i++) {
    joint_angles[i]      = 0.0;
    last_joint_angles[i] = 0.0;
  }
  ROS_INFO("[ArmManager]: Waiting for arm feedback...");

  while (!getting_joint_angles || !getting_effector_pos) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  is_initialized = true;
  ROS_INFO("[ArmManager]: Initialized Kinova Arm type: %s", arm_type.c_str());
}
//}

/* callbackHomingService //{ */
bool ArmManager::callbackHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Reset last arm goal. Homing...");
  status    = MotionStatus_t::HOMING;
  last_goal = home_pose;

  kinova_msgs::HomeArm msg;
  service_client_homing.call(msg.request, msg.response);
  res.success = true;
  return true;
}
//}

/* callbackSoftHomingService //{ */
bool ArmManager::callbackSoftHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Reset last arm goal. Soft homing...");
  status    = MotionStatus_t::HOMING;
  last_goal = home_pose;
  goTo(home_pose);

  res.success = true;
  return true;
}
//}

/* callbackPrepareGrippingService //{ */
bool ArmManager::callbackPrepareGrippingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Assuming a default gripping pose");
  status    = MotionStatus_t::MOVING;
  last_goal = default_gripping_pose;
  goTo(default_gripping_pose);

  res.success = true;
  return true;
}
//}

/* callbackGoToService //{ */
bool ArmManager::callbackGoToService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res) {

  std::scoped_lock lock(arm_state_mutex);
  if (!is_initialized) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goTo\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goTo\", not initialized!";
    return true;
  }

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goTo\", internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != MotionStatus_t::IDLE) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goTo\", arm is not IDLE!");
    res.success = false;
    res.message = "Cannot execute \"goTo\", arm is not IDLE!";
    return true;
  }

  Pose3d pose;
  pose.pos = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  status   = MotionStatus_t::MOVING;
  goTo(pose);
  res.success = true;
  return true;
}
//}

/* callbackGoToRelativeService //{ */
bool ArmManager::callbackGoToRelativeService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res) {

  std::scoped_lock lock(arm_state_mutex);
  if (!is_initialized) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goToRelative\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", not initialized!";
    return true;
  }

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goToRelative\", internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != MotionStatus_t::IDLE) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goToRelative\", arm is not IDLE!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", arm is not IDLE!";
    return true;
  }

  Pose3d pose;
  pose.pos = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  status   = MotionStatus_t::MOVING;
  goToRelative(pose);
  res.success = true;
  return true;
}
//}

/* callbackMoveDownUntilObstacleService //{ */
bool ArmManager::callbackMoveDownUntilObstacleService(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::scoped_lock lock(arm_state_mutex);
  if (!is_initialized) {
    ROS_ERROR("[ArmManager]: Cannot set end effector velocity, not initialized!");
    res.success = false;
    res.message = "Cannot set end effector velocity, not initialized!";
    return true;
  }

  if (!getting_effector_pos || !getting_joint_angles) {
    ROS_ERROR("[ArmManager]: Cannot set end effector velocity, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != MotionStatus_t::IDLE) {
    ROS_ERROR("[ArmManager]: Cannot set end effector velocity, arm is not IDLE!");
    res.success = false;
    res.message = "Cannot set end effector velocity, arm is not IDLE!";
    return true;
  }

  Eigen::Vector3d vel(0.0, 0.0, -0.3);
  status = MotionStatus_t::MOVING;
  setVelocity(vel);
  res.success = true;
  return true;
}
//}

/* callbackJointAnglesTopic //{ */
void ArmManager::callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg) {
  std::scoped_lock lock(arm_state_mutex);
  getting_joint_angles = true;
  joint_angles[0]      = msg->joint1;
  joint_angles[1]      = msg->joint2;
  joint_angles[2]      = msg->joint3;
  joint_angles[3]      = msg->joint4;
  joint_angles[4]      = msg->joint5;
  joint_angles[5]      = msg->joint6;
}
//}

/* callbackEndEffectorPoseTopic //{ */
void ArmManager::callbackEndEffectorPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg) {
  std::scoped_lock lock(arm_state_mutex);
  getting_effector_pos      = true;
  end_effector_pose.pos.x() = msg->pose.position.x;
  end_effector_pose.pos.y() = msg->pose.position.y;
  end_effector_pose.pos.z() = msg->pose.position.z;

  end_effector_pose.rot.w() = msg->pose.orientation.w;
  end_effector_pose.rot.x() = msg->pose.orientation.x;
  end_effector_pose.rot.y() = msg->pose.orientation.y;
  end_effector_pose.rot.z() = msg->pose.orientation.z;
}
//}

/* goTo //{ */
void ArmManager::goTo(Pose3d pose) {
  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[ArmManager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(), euler[0],
           euler[1], euler[2]);
  last_goal = pose;
  kinova_msgs::ArmPoseActionGoal msg;
  msg.goal.pose.pose.position.x = pose.pos.x();
  msg.goal.pose.pose.position.y = pose.pos.y();
  msg.goal.pose.pose.position.z = pose.pos.z();

  msg.goal.pose.pose.orientation.x = pose.rot.x();
  msg.goal.pose.pose.orientation.y = pose.rot.y();
  msg.goal.pose.pose.orientation.z = pose.rot.z();
  msg.goal.pose.pose.orientation.w = pose.rot.w();

  std::stringstream ss;
  ss << arm_type << "_link_base";
  msg.goal.pose.header.frame_id = ss.str().c_str();
  msg.header.frame_id           = ss.str().c_str();

  publisher_end_effector_pose.publish(msg);
}
//}

/* goToRelative //{ */
void ArmManager::goToRelative(Pose3d rel_pose) {

  // add rel_pose to the current end effector pose
  Pose3d pose = end_effector_pose + rel_pose;

  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[ArmManager]: Moving end effector by relative [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(), euler[0],
           euler[1], euler[2]);

  last_goal = pose;
  kinova_msgs::ArmPoseActionGoal msg;

  msg.goal.pose.pose.position.x = pose.pos.x();
  msg.goal.pose.pose.position.y = pose.pos.y();
  msg.goal.pose.pose.position.z = pose.pos.z();

  msg.goal.pose.pose.orientation.x = pose.rot.x();
  msg.goal.pose.pose.orientation.y = pose.rot.y();
  msg.goal.pose.pose.orientation.z = pose.rot.z();
  msg.goal.pose.pose.orientation.w = pose.rot.w();

  std::stringstream ss;
  ss << arm_type << "_link_base";
  msg.goal.pose.header.frame_id = ss.str().c_str();
  msg.header.frame_id           = ss.str().c_str();

  publisher_end_effector_pose.publish(msg);
}
//}

/* setVelocity //{ */
void ArmManager::setVelocity(Eigen::Vector3d cartesian_velocity) {

  ROS_INFO("[ArmManager]: Set end effector velocity [%.3f, %.3f, %.3f]", cartesian_velocity.x(), cartesian_velocity.y(), cartesian_velocity.z());
  kinova_msgs::PoseVelocity msg;
  msg.twist_linear_x = cartesian_velocity.x();
  msg.twist_linear_y = cartesian_velocity.y();
  msg.twist_linear_z = cartesian_velocity.z();

  publisher_cartesian_velocity.publish(msg);
}
//}

/* statusTimer //{ */
void ArmManager::statusTimer([[maybe_unused]] const ros::TimerEvent &evt) {
  if (!is_initialized) {
    return;
  }
  kinova_control_manager::ArmStatus status_msg;
  status_msg.arm_type     = arm_type;
  status_msg.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << arm_type << "_link_base";
  status_msg.header.frame_id = ss.str().c_str();
  status_msg.status          = statusToString(status);
  for (int i = 0; i < DOF; i++) {
    status_msg.joint_angles[i] = joint_angles[i];
  }

  Eigen::Vector3d euler      = quaternionToEuler(end_effector_pose.rot);
  Eigen::Vector3d last_euler = quaternionToEuler(last_goal.rot);
  for (int i = 0; i < 3; i++) {
    status_msg.end_effector_pose[i]     = end_effector_pose.pos[i];
    status_msg.end_effector_pose[i + 3] = euler[i];
    status_msg.last_goal[i]             = last_goal.pos[i];
    status_msg.last_goal[i + 3]         = last_euler[i];
  }

  publisher_arm_status.publish(status_msg);
}
//}

/* motionCheckTimer //{ */
void ArmManager::motionCheckTimer([[maybe_unused]] const ros::TimerEvent &evt) {

  if (!is_initialized) {
    return;
  }

  time_since_last_movement += 1.0 / motion_check_timer_rate;

  if (status == MotionStatus_t::IDLE) {
    updateLastJointAngles();
    return;
  }

  if (status == MotionStatus_t::HOMING) {
    if (nearbyPose(end_effector_pose, home_pose)) {
      ROS_INFO("[ArmManager]: Homing complete");
      status = MotionStatus_t::IDLE;
    }
    updateLastJointAngles();
    return;
  }

  bool motion_stopped = true;
  for (int i = 0; i < DOF; i++) {
    motion_stopped = motion_stopped && nearbyAngleDeg(joint_angles[i], last_joint_angles[i]);
  }

  if (!motion_stopped) {
    time_since_last_movement = 0.0;
  }

  if (motion_stopped && time_since_last_movement > no_move_error_timeout) {
    status = MotionStatus_t::IDLE;
    if (nearbyPose(end_effector_pose, last_goal)) {
      ROS_INFO("[Arm manager]: Goal reached");
    } else {
      ROS_WARN("[Arm manager]: Goal unreachable!");
    }
  }
  updateLastJointAngles();
  return;
}
//}

/* updateLastJointAngles //{ */
void ArmManager::updateLastJointAngles() {

  std::scoped_lock lock(arm_state_mutex);
  for (int i = 0; i < DOF; i++) {
    last_joint_angles[i] = joint_angles[i];
  }
}
//}

/* statusToString //{ */
std::string ArmManager::statusToString(MotionStatus_t ms) {
  switch (ms) {
    case MotionStatus_t::IDLE:
      return "IDLE";
    case MotionStatus_t::MOVING:
      return "MOVING";
    case MotionStatus_t::HOMING:
      return "HOMING";
  }
}
//}

/* nearbyPose //{ */
bool ArmManager::nearbyPose(Pose3d p, Pose3d q) {
  /* double pos_diff = (p.pos - q.pos).norm(); */
  /* std::cout << "Pos diff: " << pos_diff << "\n"; */

  Eigen::Vector3d p_euler = quaternionToEuler(p.rot);
  Eigen::Vector3d q_euler = quaternionToEuler(q.rot);
  Eigen::Vector3d angular_diff;

  for (int i = 0; i < 3; i++) {
    angular_diff[i] = std::abs(p_euler[i] - q_euler[i]);
  }
  /* std::cout << "Angular diff: " << angular_diff[0] << ", " << angular_diff[1] << ", " << angular_diff[2] << "\n"; */

  return (p.pos - q.pos).norm() < nearby_position_threshold && angular_diff[0] < nearby_rotation_threshold && angular_diff[1] < nearby_rotation_threshold &&
         angular_diff[2] < nearby_rotation_threshold;
}
//}

/* nearbyAngleDeg //{ */
bool ArmManager::nearbyAngleDeg(float a, float b) {
  return (std::abs(b - a) * M_PI) / 180 < nearby_rotation_threshold;
}
//}

/* nearbyAngleDeg //{ */
bool ArmManager::nearbyAngleRad(float a, float b) {
  return std::abs(b - a) < nearby_rotation_threshold;
}
//}

/* quaternionToEuler //{ */
Eigen::Vector3d ArmManager::quaternionToEuler(Eigen::Quaterniond q) {
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}
//}

/* eulerToQuaternion //{ */
Eigen::Quaterniond ArmManager::eulerToQuaternion(Eigen::Vector3d euler) {
  return Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
}
//}

}  // namespace kinova_control_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kinova_control_manager::ArmManager, nodelet::Nodelet)
