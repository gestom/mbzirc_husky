#include <random>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>

#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#include <actionlib/client/simple_action_client.h>

#include <mbzirc_husky_msgs/Gen3ArmStatus.h>
#include <mbzirc_husky_msgs/EndEffectorPose.h>
#include <mbzirc_husky_msgs/Vector7.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <mbzirc_husky_msgs/StoragePosition.h>

#include <sensor_msgs/JointState.h>
#include <kortex_driver/TwistCommand.h>
#include <pouring_msgs/MoveGroupAction.h>

#include <mrs_msgs/GripperDiagnostics.h>

#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>

#define DOF 7
#define ZERO_VELOCITY Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0)

typedef enum
{
  MOVING,
  IDLE,
  HOMING,
  ALIGNING,
} MotionStatus;

typedef enum
{
  RUNNING,
  COMPLETED,
  FAILED
} ActionStatus;

/* utils //{ */

/* quaternionToEuler //{ */
Eigen::Vector3d quaternionToEuler(Eigen::Quaterniond q) {
  return q.toRotationMatrix().eulerAngles(0, 1, 2);
}
//}

/* eulerToQuaternion //{ */
Eigen::Quaterniond eulerToQuaternion(Eigen::Vector3d euler) {
  return Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
}
//}

/* statusToString //{ */
std::string statusToString(MotionStatus ms) {
  switch (ms) {
    case IDLE:
      return "IDLE";
    case MOVING:
      return "MOVING";
    case HOMING:
      return "HOMING";
    case ALIGNING:
      return "ALIGNING";
  }
}
//}

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

/* struct Brick //{ */
struct Brick
{
  Pose3d pose;
  int    brick_class;
  int    brick_layer;
};
//}

//}

/* VARIABLES //{ */

// params
double arm_base_to_ground;
double gripper_length;
double gripper_timeout;
double no_move_joint_velocity;
double move_down_speed_faster;
double move_down_speed_slower;
double nearby_rot_threshold;
double nearby_pos_threshold;
double linear_vel_modifier;
double linear_vel_max;
double linear_vel_min;
double angular_vel_modifier;
double angular_vel_max;
double angular_vel_min;
double align_timeout;
double descent_to_storage;
double gripper_threshold;

Eigen::Vector3d camera_offset;


// continuous status publishing
ros::Timer status_timer;
double     status_timer_rate;
void       statusTimer(const ros::TimerEvent &evt);

// motion control with planner feedback
bool goToAction(Pose3d pose);
bool goToAnglesAction(Pose3d pose);

// predefined positions
std::vector<double> home_angles;
std::vector<double> raised_camera_angles;
Pose3d              gripping_pose;

std::vector<Pose3d> storage_poses;

// arm status
bool is_initialized            = false;
bool brick_attached            = false;
bool gripper_on                = false;
bool getting_joint_angles      = false;
bool getting_end_effector_pose = false;
bool getting_gripper_feedback  = false;
bool getting_realsense_brick   = false;
bool brick_reliable            = false;

std::vector<double> joint_angles;
Pose3d              end_effector_pose;

MotionStatus status;
Brick        detected_brick;

ros::Time last_brick_time;
ros::Time gripper_start_time;

// action client
std::unique_ptr<actionlib::SimpleActionClient<pouring_msgs::MoveGroupAction>> action_client_goto_;

// advertised services
ros::ServiceServer service_server_align_arm;
ros::ServiceServer service_server_align_arm_goto;
ros::ServiceServer service_server_goto;
ros::ServiceServer service_server_goto_relative;
ros::ServiceServer service_server_goto_angles;
ros::ServiceServer service_server_goto_angles_relative;
ros::ServiceServer service_server_goto_storage;
ros::ServiceServer service_server_homing;
ros::ServiceServer service_server_lift_brick;
ros::ServiceServer service_server_prepare_gripping;
ros::ServiceServer service_server_pickup_brick;
ros::ServiceServer service_server_raise_camera;
ros::ServiceServer service_server_store_brick;
ros::ServiceServer service_server_unload_brick;

// called services
ros::ServiceClient service_client_grip;
ros::ServiceClient service_client_ungrip;

// publishers
ros::Publisher publisher_arm_status;
ros::Publisher publisher_camera_to_ground;
ros::Publisher publisher_rviz_markers;
ros::Publisher publisher_cartesian_velocity;

// subscribers
ros::Subscriber subscriber_joint_state;
ros::Subscriber subscriber_brick_pose;
ros::Subscriber subscriber_gripper_diagnostics;

// tf
std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

ActionStatus action_status_;
bool         action_successful_;

//}

/* nearbyAngle //{ */
bool nearbyAngle(double a, double b, double threshold = nearby_rot_threshold) {
  return std::abs(a - b) < threshold;
}
//}

/* nearbyAngles //{ */
bool nearbyAngles(std::vector<double> u, std::vector<double> v, double threshold = nearby_rot_threshold) {
  for (int i = 0; i < DOF; i++) {
    if (std::abs(u[i] - v[i]) > threshold) {
      return false;
    }
  }
  return true;
}
//}

/* nearbyPose //{ */
bool nearbyPose(Pose3d p, Pose3d q) {
  double pos_diff     = (p.pos - q.pos).norm();
  double angular_diff = p.rot.angularDistance(q.rot);
  return (pos_diff < nearby_pos_threshold) && (angular_diff < nearby_rot_threshold);
}
//}

/* grip //{ */
bool grip() {
  std_srvs::Trigger trig;
  service_client_grip.call(trig.request, trig.response);
  return trig.response.success;
}
//}

/* ungrip //{ */
bool ungrip() {
  std_srvs::Trigger trig;
  service_client_ungrip.call(trig.request, trig.response);
  ROS_INFO_STREAM("[" << ros::this_node::getName().c_str() << "]: " << trig.response.message);
  return trig.response.success;
}
//}

/* arm action monitoring //{ */
void actionDoneCb(const actionlib::SimpleClientGoalState &state, const pouring_msgs::MoveGroupResultConstPtr &result) {
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  if (result->result) {
    action_status_ = COMPLETED;
  } else {
    ROS_ERROR("[%s]", result->error_msg.c_str());
    ROS_ERROR("[%s]: Destination unreachable!", ros::this_node::getName().c_str());
    action_status_ = FAILED;
  }
}

// Called once when the goal becomes active
void actionActiveCb() {
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void actionFeedbackCb([[maybe_unused]] const pouring_msgs::MoveGroupFeedbackConstPtr &feedback) {
  ROS_INFO("Got Feedback");
}
//}

/* goToAction//{ */
// Called once when the goal completes
bool goToAction(Pose3d goal_pose) {

  Eigen::Vector3d goal_euler = quaternionToEuler(goal_pose.rot);
  ROS_INFO("[%s]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", ros::this_node::getName().c_str(), goal_pose.pos.x(),
           goal_pose.pos.y(), goal_pose.pos.z(), goal_euler.x(), goal_euler.y(), goal_euler.z());

  pouring_msgs::MoveGroupGoal goal;
  goal.path_constraint = false;
  goal.cartesian       = true;
  goal.relative        = false;

  goal.pose_reference_frame         = "base_link";
  goal.cartesian_goal.position.x    = goal_pose.pos.x();
  goal.cartesian_goal.position.y    = goal_pose.pos.y();
  goal.cartesian_goal.position.z    = goal_pose.pos.z();
  goal.cartesian_goal.orientation.w = goal_pose.rot.w();
  goal.cartesian_goal.orientation.x = goal_pose.rot.x();
  goal.cartesian_goal.orientation.y = goal_pose.rot.y();
  goal.cartesian_goal.orientation.z = goal_pose.rot.z();

  action_status_ = RUNNING;
  action_client_goto_->sendGoal(goal, &actionDoneCb, &actionActiveCb, &actionFeedbackCb);

  while (action_status_ == RUNNING) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
  status = IDLE;

  if (action_status_ == COMPLETED) {
    ROS_INFO("Action completed");
    return true;
  } else {
    ROS_WARN("Action failed");
    return false;
  }

  return false;
}
//}

/* goToAnglesAction //{ */
// Called once when the goal completes

bool goToAnglesAction(std::vector<double> angles) {

  ROS_INFO("[%s]: Moving end effector to position [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", ros::this_node::getName().c_str(), angles[0], angles[1],
           angles[2], angles[3], angles[4], angles[5], angles[6]);

  pouring_msgs::MoveGroupGoal goal;
  for (int i = 0; i < DOF; i++) {
    goal.target_joint_values.push_back(angles[i]);
  }
  goal.pose_reference_frame    = "world";
  goal.path_constraint         = false;
  goal.use_current_orientation = false;
  goal.cartesian               = false;
  goal.relative                = false;

  action_status_ = RUNNING;
  action_client_goto_->sendGoal(goal, &actionDoneCb, &actionActiveCb, &actionFeedbackCb);

  while (action_status_ == RUNNING) {
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
  status = IDLE;

  if (action_status_ == COMPLETED) {
    ROS_INFO("Action completed");
    return true;
  } else {
    ROS_WARN("Action failed");
    return false;
  }

  return false;
}
//}

/* setCartesianVelocity //{ */
void setCartesianVelocity(Eigen::Vector3d linear, Eigen::Vector3d angular) {
  ROS_INFO("[%s]: Setting cartesian velocity: linear [%.2f, %.2f, %.2f], angular: [%.2f, %.2f, %.2f]", ros::this_node::getName().c_str(), linear.x(),
           linear.y(), linear.z(), angular.x(), angular.y(), angular.z());
  kortex_driver::TwistCommand msg;
  msg.twist.linear_x  = linear.x();
  msg.twist.linear_y  = linear.y();
  msg.twist.linear_z  = linear.z();
  msg.twist.angular_x = angular.x();
  msg.twist.angular_y = angular.y();
  msg.twist.angular_z = angular.z();
  publisher_cartesian_velocity.publish(msg);
}
//}

/* callbackAlignArmService //{ */
bool callbackAlignArmService([[maybe_unused]] std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot align_arm, not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot align_arm, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot align_arm, not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_realsense_brick) {
    ROS_FATAL("[%s]: Brick pose topic did not open!", ros::this_node::getName().c_str());
    status      = IDLE;
    res.success = false;
    return false;
  }

  status = ALIGNING;

  bool aligned = false;

  Eigen::Vector3d linear_vel;
  Eigen::Vector3d angular_vel;

  ros::Time align_start = ros::Time::now();
  while (!aligned && (ros::Time::now() - align_start).toSec() < align_timeout) {
    Eigen::Vector3d brick_euler = quaternionToEuler(detected_brick.pose.rot);
    double          align_x     = -(detected_brick.pose.pos.y() - camera_offset.x());
    double          align_y     = -(detected_brick.pose.pos.x() - camera_offset.y());

    if (end_effector_pose.pos.x() > 0.6 || end_effector_pose.pos.x() < 0.2 || end_effector_pose.pos.y() > 0.25 || end_effector_pose.pos.y() < -0.25) {
      ROS_ERROR("[%s]: Brick unreachable!", ros::this_node::getName().c_str());
      status = IDLE;
      setCartesianVelocity(ZERO_VELOCITY);
      res.success = false;
      return false;
    }

    linear_vel.x()  = align_x * linear_vel_modifier;
    linear_vel.y()  = align_y * linear_vel_modifier;
    linear_vel.z()  = 0.0;
    angular_vel.x() = 0.0;
    angular_vel.y() = 0.0;
    angular_vel.z() = brick_euler.z() * angular_vel_modifier;

    if (linear_vel.norm() > linear_vel_max) {
      linear_vel.normalize();
      linear_vel = linear_vel_max * linear_vel;
    }
    if (linear_vel.norm() < 0.03) {
      linear_vel.normalize();
      linear_vel = linear_vel * 0.03;
    }
    if (angular_vel.norm() > angular_vel_max) {
      angular_vel.normalize();
      angular_vel = angular_vel_max * angular_vel;
    }

    setCartesianVelocity(linear_vel, angular_vel);
    aligned = std::abs(align_x) < nearby_pos_threshold && std::abs(align_y) < nearby_pos_threshold && std::abs(brick_euler.z()) < nearby_rot_threshold;
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  setCartesianVelocity(ZERO_VELOCITY);
  status      = IDLE;
  res.success = aligned;
  return aligned;
}
//}

/* callbackGoToService //{ */
bool callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot execute \"goTo\", not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot execute \"goTo\", internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot execute \"goTo\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;
  Pose3d pose;
  pose.pos          = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot          = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  bool goal_reached = goToAction(pose);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToRelativeService //{ */
bool callbackGoToRelativeService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot execute \"goToRelative\", not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot execute \"goToRelative\", internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot execute \"goToRelative\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;

  Pose3d rel_pose;
  rel_pose.pos = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  rel_pose.rot = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));


  bool goal_reached = goToAction(end_effector_pose + rel_pose);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToAnglesService //{ */
bool callbackGoToAnglesService(mbzirc_husky_msgs::Vector7Request &req, mbzirc_husky_msgs::Vector7Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot execute \"goToAngles\", not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot execute \"goToAngles\", internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot execute \"goToAngles\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;

  std::vector<double> goal_angles;
  for (int i = 0; i < DOF; i++) {
    goal_angles.push_back(req.data[i]);
  }

  bool goal_reached = goToAnglesAction(goal_angles);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToAnglesRelativeService //{ */

bool callbackGoToAnglesRelativeService(mbzirc_husky_msgs::Vector7Request &req, mbzirc_husky_msgs::Vector7Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot execute \"goToAnglesRelative\", not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot execute \"goToAnglesRelative\", internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot execute \"goToAnglesRelative\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;

  std::vector<double> goal_angles;
  for (int i = 0; i < DOF; i++) {
    goal_angles.push_back(req.data[i] + joint_angles[i]);
  }

  bool goal_reached = goToAnglesAction(goal_angles);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToStorageService //{ */
bool callbackGoToStorageService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot execute \"goToStorage\", not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot execute \"goToStorage\", internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot execute \"goToStorage\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("[%s]: Moving arm to storage position %d, layer %d", ros::this_node::getName().c_str(), req.position, req.layer);
  bool have_brick = brick_attached;
  status          = MOVING;

  Pose3d goal_pose = storage_poses[req.position];

  bool goal_reached = goToAction(goal_pose);

  if (have_brick != brick_attached) {
    ROS_ERROR("[%s]: Brick lost during motion!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackHomingService //{ */
bool callbackHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot move, not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }
  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot move, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("[%s]: Reset last arm goal. Homing...", ros::this_node::getName().c_str());
  status = HOMING;

  ungrip();
  bool goal_reached = goToAnglesAction(home_angles);

  status      = IDLE;
  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackLiftBrickService //{ */
bool callbackLiftBrickService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot move, not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot move, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot move, arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("[%s]: Lifting brick up", ros::this_node::getName().c_str());
  status = MOVING;

  Pose3d goal_pose;
  goal_pose.pos = gripping_pose.pos;
  goal_pose.pos.z() += req.data;
  goal_pose.rot = gripping_pose.rot;

  bool goal_reached = goToAction(goal_pose);

  if (!brick_attached) {
    ROS_ERROR("[%s]: Brick lost during ascent!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackPickupBrickService //{ */
bool callbackPickupBrickService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot start \"pickup brick\", arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_realsense_brick || !getting_gripper_feedback) {
    ROS_WARN("[%s]: Cannot pickup brick whithout realsense and gripper feedback!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;

  ROS_INFO("[%s]: Moving down for brick at layer %d", ros::this_node::getName().c_str(), detected_brick.brick_layer);

  Eigen::Vector3d linear_vel;
  Eigen::Vector3d angular_vel;

  double align_x, align_y;
  while (!brick_attached && brick_reliable) {

    if (!getting_realsense_brick) {
      ROS_ERROR("[%s]: Brick lost, aborting pickup", ros::this_node::getName().c_str());
      setCartesianVelocity(ZERO_VELOCITY);
      status      = IDLE;
      res.success = false;
      return false;
    }
    Eigen::Vector3d brick_euler = quaternionToEuler(detected_brick.pose.rot);

    align_x = -detected_brick.pose.pos.y();
    align_y = -detected_brick.pose.pos.x();

    linear_vel.x()  = align_x * linear_vel_modifier;
    linear_vel.y()  = align_y * linear_vel_modifier;
    linear_vel.z()  = -move_down_speed_faster;
    angular_vel.x() = 0.0;
    angular_vel.y() = 0.0;
    angular_vel.z() = brick_euler.z() * angular_vel_modifier;

    if (linear_vel.norm() > linear_vel_max) {
      linear_vel.normalize();
      linear_vel = linear_vel_max * linear_vel;
    }

    if (linear_vel.norm() < linear_vel_min) {
      linear_vel.normalize();
      linear_vel = linear_vel_min * linear_vel;
    }

    if (angular_vel.norm() > angular_vel_max) {
      angular_vel.normalize();
      angular_vel = angular_vel_max * angular_vel;
    }

    if (angular_vel.norm() < angular_vel_min) {
      angular_vel.normalize();
      angular_vel = angular_vel_min * angular_vel;
    }

    setCartesianVelocity(linear_vel, angular_vel);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  ROS_INFO("[%s]: Brick unreliable, going straight down", ros::this_node::getName().c_str());

  double stopping_height = (detected_brick.brick_layer * 0.2) - 0.04;
  ROS_INFO("[%s]: Stopping height: %.2f", ros::this_node::getName().c_str(), stopping_height);

  grip();
  double magnet_to_ground = end_effector_pose.pos.z() - camera_offset.z() + arm_base_to_ground;

while (!brick_attached){ // && magnet_to_ground > stopping_height) {
    magnet_to_ground = end_effector_pose.pos.z() + arm_base_to_ground - gripper_length;
    ROS_INFO("[%s]: Magnet to ground: %.2f", ros::this_node::getName().c_str(), magnet_to_ground);

    /*
    if (status == IDLE) {
      ROS_ERROR("[%s]: Arm stopped! Pickup failed", ros::this_node::getName().c_str());
      setCartesianVelocity(ZERO_VELOCITY);
      res.success = false;
      return false;
    }
    */

    linear_vel.x()  = 0.0;
    linear_vel.y()  = 0.0;
    linear_vel.z()  = -move_down_speed_slower;
    angular_vel.x() = 0.0;
    angular_vel.y() = 0.0;
    angular_vel.z() = 0.0;
    setCartesianVelocity(linear_vel, angular_vel);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  status = IDLE;
  setCartesianVelocity(ZERO_VELOCITY);
  usleep(10000);
  status = IDLE;
  if (brick_attached) {
    ROS_INFO("[%s]: Brick attached", ros::this_node::getName().c_str());
    return true;
  }
  return false;
}
//}

/* callbackPrepareGrippingService //{ */
bool callbackPrepareGrippingService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot move, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {

    if (nearbyPose(gripping_pose, end_effector_pose)) {
      ROS_INFO("[%s]: Assumed a default gripping pose", ros::this_node::getName().c_str());
      status      = IDLE;
      res.success = true;
      return true;
    }

    ROS_ERROR("[%s]: Cannot move, arm is not IDLE", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("[%s]: Assuming a default gripping pose", ros::this_node::getName().c_str());
  status = MOVING;

  Pose3d goal_pose = gripping_pose;
  goal_pose.pos.z() += req.data;

  bool goal_reached = goToAction(goal_pose);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackRaiseCameraService //{ */
bool callbackRaiseCameraService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot move, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot move, arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  ROS_INFO("[%s]: Assuming a raised camera pose", ros::this_node::getName().c_str());
  status            = MOVING;
  bool goal_reached = goToAnglesAction(raised_camera_angles);

  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackStoreBrickService //{ */
bool callbackStoreBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {
  if (!is_initialized) {
    ROS_ERROR("[%s]: Cannot store brick, not initialized!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[%s]: Cannot store brick, internal arm feedback missing!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[%s]: Cannot store brick, arm is not IDLE!", ros::this_node::getName().c_str());
    res.success = false;
    return false;
  }

  status = MOVING;

  Pose3d new_goal = storage_poses[req.position];
  new_goal.pos.z() += (0.2 * req.layer) - descent_to_storage;
  bool goal_reached = goToAction(new_goal);
  ungrip();
  ros::Duration(0.2).sleep();
  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackUnloadBrickService //{ */
bool callbackUnloadBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {
  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot unload brick, not initialized!");
    res.success = false;
    res.message = "Cannot unload brick from storage, not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot unload brick, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot unload brick, arm is not IDLE!");
    res.success = false;
    res.message = "Cannot unload brick, arm is not IDLE!";
    return false;
  }

  status = MOVING;

  Pose3d new_goal = storage_poses[req.position];
  new_goal.pos.z() += (0.2 * req.layer) - (0.75 * descent_to_storage);
  goToAction(new_goal);
  grip();
  ROS_INFO("[%s]: Switching to velocity control until magnet grips a brick", ros::this_node::getName().c_str());

  Eigen::Vector3d linear_vel;
  Eigen::Vector3d angular_vel;
  while (!brick_attached) {  // TODO add some safety mechanism
    linear_vel.x()  = 0.0;
    linear_vel.y()  = 0.0;
    linear_vel.z()  = -move_down_speed_slower;
    angular_vel.x() = 0.0;
    angular_vel.y() = 0.0;
    angular_vel.z() = 0.0;
    setCartesianVelocity(linear_vel, angular_vel);
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  setCartesianVelocity(ZERO_VELOCITY);

  if (brick_attached) {
    ROS_INFO("[%s]: Brick attached", ros::this_node::getName().c_str());
    status      = IDLE;
    res.success = true;
    return true;
  }
  status      = IDLE;
  res.success = false;
  return false;
}
//}

/* callbackBrickPoseTopic //{ */
void callbackBrickPoseTopic(const mbzirc_husky_msgs::brickPositionConstPtr &msg) {
  getting_realsense_brick = msg->detected;
  last_brick_time         = ros::Time::now();
  brick_reliable          = msg->completelyVisible && msg->detected;

  detected_brick.brick_class = msg->type;

  if (brick_reliable) {
    Eigen::Vector3d measurement(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    detected_brick.pose.pos.x() = msg->pose.pose.position.x;
    detected_brick.pose.pos.y() = msg->pose.pose.position.y;
    detected_brick.pose.pos.z() = msg->pose.pose.position.z;

    detected_brick.pose.rot.w() = msg->pose.pose.orientation.w;
    detected_brick.pose.rot.x() = msg->pose.pose.orientation.x;
    detected_brick.pose.rot.y() = msg->pose.pose.orientation.y;
    detected_brick.pose.rot.z() = msg->pose.pose.orientation.z;

    double end_effector_to_ground = end_effector_pose.pos.z() + arm_base_to_ground - gripper_length;
    double brick_height           = end_effector_to_ground - (detected_brick.pose.pos.z() + 0.2);
    /* std::cout << "Brick height: " << brick_height << "\n"; */
    if (status == ALIGNING) {
      if (brick_height > -0.15 && brick_height < 0.15) {
        detected_brick.brick_layer = 1;
      } else if (brick_height >= 0.15 && brick_height < 0.35) {
        detected_brick.brick_layer = 2;
      } else if (brick_height >= 0.35) {
        detected_brick.brick_layer = 3;
      }
      /* std::cout << "Layer: " << detected_brick.brick_layer << "\n"; */
    }
  }
}
//}

/* callbackJointStateTopic //{ */
void callbackJointStateTopic(const sensor_msgs::JointStateConstPtr &msg) {
  getting_joint_angles = true;

  for (int i = 0; i < DOF; i++) {
    joint_angles[i] = msg->position[i];
  }

  try {
    auto lookuptf = tf_buffer_->lookupTransform("base_link", "end_effector_link", ros::Time(0));

    end_effector_pose.pos.x() = lookuptf.transform.translation.x;
    end_effector_pose.pos.y() = lookuptf.transform.translation.y;
    end_effector_pose.pos.z() = lookuptf.transform.translation.z;
    end_effector_pose.rot.w() = lookuptf.transform.rotation.w;
    end_effector_pose.rot.x() = lookuptf.transform.rotation.x;
    end_effector_pose.rot.y() = lookuptf.transform.rotation.y;
    end_effector_pose.rot.z() = lookuptf.transform.rotation.z;
    getting_end_effector_pose = true;
  }

  catch (tf2::LookupException ex) {
    ROS_ERROR("[%s]: %s", ros::this_node::getName().c_str(), ex.what());
  }

  if (status != IDLE) {
    bool stopped = true;
    for (int i = 0; i < DOF; i++) {
      stopped = stopped && msg->velocity[i] < no_move_joint_velocity;
    }
    if (stopped) {
      ROS_WARN("[%s]: Motion stopped! Is the arm pushing something?", ros::this_node::getName().c_str());
    }
  }
}
//}

/* callbackGripperDiagnosticsTopic //{ */
void callbackGripperDiagnosticsTopic(const mrs_msgs::GripperDiagnosticsConstPtr &msg) {
  getting_gripper_feedback = true;
  if (!gripper_on && msg->gripper_on) {
    gripper_start_time = ros::Time::now();
  }
  gripper_on     = msg->gripper_on;

  if (msg->hall1_debug < gripper_threshold){
  	brick_attached = true;
  }else{
  	brick_attached = false;
  }
}
//}

/* publishVisualMarkers //{ */
void publishVisualMarkers() {
  if (brick_reliable) {
    visualization_msgs::Marker msg;
    msg.header.frame_id    = "end_effector_link";
    msg.header.stamp       = ros::Time::now();
    msg.ns                 = "brick_namespace";
    msg.action             = visualization_msgs::Marker::ADD;
    msg.type               = visualization_msgs::Marker::CUBE;
    msg.id                 = 666;
    msg.pose.position.x    = detected_brick.pose.pos.y();
    msg.pose.position.y    = detected_brick.pose.pos.x();
    msg.pose.position.z    = detected_brick.pose.pos.z() + 0.1;
    msg.pose.orientation.x = detected_brick.pose.rot.x();
    msg.pose.orientation.y = detected_brick.pose.rot.y();
    msg.pose.orientation.z = detected_brick.pose.rot.z();
    msg.pose.orientation.w = detected_brick.pose.rot.w();
    switch (detected_brick.brick_class) {
      case 1:
        msg.scale.x = 0.3;
        msg.color.r = 1.0;
        break;
      case 2:
        msg.scale.x = 0.6;
        msg.color.g = 1.0;
        break;
      case 3:
        msg.scale.x = 1.2;
        msg.color.b = 1.0;
        break;
      default:
        return;
    }
    msg.scale.y = 0.2;
    msg.scale.z = 0.2;
    msg.color.a = 0.4;
    publisher_rviz_markers.publish(msg);
  }
}
//}

/* statusTimer //{ */
void statusTimer([[maybe_unused]] const ros::TimerEvent &evt) {

  if (!is_initialized) {
    return;
  }
  if (!getting_end_effector_pose || !getting_joint_angles) {
    return;
  }

  mbzirc_husky_msgs::Gen3ArmStatus status_msg;
  status_msg.arm_type     = "my_gen3";
  status_msg.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << "my_gen3_link_base";
  status_msg.header.frame_id = ss.str().c_str();
  status_msg.status          = statusToString(status);
  for (int i = 0; i < DOF; i++) {
    status_msg.joint_angles[i] = joint_angles[i];
  }
  status_msg.gripper_on     = gripper_on;
  status_msg.brick_attached = brick_attached;

  if (gripper_on && !brick_attached && (ros::Time::now() - gripper_start_time).toSec() > gripper_timeout) {
    ROS_WARN("[%s]: Gripper timeout! Turning the gripper off!", ros::this_node::getName().c_str());
    ungrip();
  }

  Eigen::Vector3d euler = quaternionToEuler(end_effector_pose.rot);
  for (int i = 0; i < 3; i++) {
    status_msg.end_effector_pose[i]     = end_effector_pose.pos[i];
    status_msg.end_effector_pose[i + 3] = euler[i];
  }

  if (status == ALIGNING && (ros::Time::now() - last_brick_time).sec > 2.0) {
    ROS_WARN("[%s]: Brick timeout!", ros::this_node::getName().c_str());
    getting_realsense_brick     = false;
    detected_brick.pose.pos.x() = 0.0;
    detected_brick.pose.pos.y() = 0.0;
    status                      = IDLE;
  }

  publisher_arm_status.publish(status_msg);

  std_msgs::Float64 cam_to_ground;
  cam_to_ground.data = end_effector_pose.pos.z() + arm_base_to_ground;
  publisher_camera_to_ground.publish(cam_to_ground);
  /* publishVisualMarkers(); */
}
//}

/* main //{ */
int main(int argc, char **argv) {
  ros::init(argc, argv, "kortex_control_manager");
  ros::NodeHandle nh = ros::NodeHandle("~");

  ROS_INFO("[%s]: Initializing...", ros::this_node::getName().c_str());

  // param containers
  std::vector<double> storage_poses_raw;
  std::vector<double> camera_offset_raw;
  std::vector<double> gripping_pose_raw;

  nh.getParam("arm_base_to_ground", arm_base_to_ground);
  nh.getParam("gripper_length", gripper_length);
  nh.getParam("gripper_timeout", gripper_timeout);
  nh.getParam("no_move_joint_velocity", no_move_joint_velocity);
  nh.getParam("move_down_speed_faster", move_down_speed_faster);
  nh.getParam("move_down_speed_slower", move_down_speed_slower);
  nh.getParam("nearby_pos_threshold", nearby_pos_threshold);
  nh.getParam("nearby_rot_threshold", nearby_rot_threshold);
  nh.getParam("home_angles", home_angles);
  nh.getParam("gripping_pose", gripping_pose_raw);
  nh.getParam("brick_storage", storage_poses_raw);
  nh.getParam("linear_vel_modifier", linear_vel_modifier);
  nh.getParam("linear_vel_max", linear_vel_max);
  nh.getParam("linear_vel_min", linear_vel_min);
  nh.getParam("angular_vel_modifier", angular_vel_modifier);
  nh.getParam("angular_vel_max", angular_vel_max);
  nh.getParam("angular_vel_min", angular_vel_min);
  nh.getParam("camera_offset", camera_offset_raw);
  nh.getParam("align_timeout", align_timeout);
  nh.getParam("status_timer_rate", status_timer_rate);
  nh.getParam("descent_to_storage", descent_to_storage);
  nh.getParam("raised_camera_angles", raised_camera_angles);
  nh.getParam("gripper_threshold", gripper_threshold);

  /* parse params //{ */
  if (gripping_pose_raw.size() != 6) {
    ROS_ERROR("[%s]: Parameter \"gripping_pose\" expected to have %d elements (value in radians for all joints), got %ld!", ros::this_node::getName().c_str(),
              DOF, gripping_pose_raw.size());
    ros::shutdown();
  }
  gripping_pose.pos.x() = gripping_pose_raw[0];
  gripping_pose.pos.y() = gripping_pose_raw[1];
  gripping_pose.pos.z() = gripping_pose_raw[2];
  gripping_pose.rot     = eulerToQuaternion(Eigen::Vector3d(gripping_pose_raw[3], gripping_pose_raw[4], gripping_pose_raw[5]));

  if (storage_poses_raw.size() % 6 != 0) {
    ROS_ERROR("[%s]: Parameter \"brick_storage\" is expected to be divisible by %d!", ros::this_node::getName().c_str(), 6);
    ros::shutdown();
  }
  int num_storage_bins = (int)(storage_poses_raw.size() / 6);
  for (int i = 0; i < num_storage_bins; i++) {
    Pose3d pose;
    pose.pos.x() = storage_poses_raw[i * 6];
    pose.pos.y() = storage_poses_raw[(i * 6) + 1];
    pose.pos.z() = storage_poses_raw[(i * 6) + 2];
    Eigen::Vector3d angles(storage_poses_raw[(i * 6) + 3], storage_poses_raw[(i * 6) + 4], storage_poses_raw[(i * 6) + 5]);
    pose.rot = eulerToQuaternion(angles);
    storage_poses.push_back(pose);
  }
  if (camera_offset_raw.size() != 3) {
    ROS_ERROR("[%s]: Paramerer \"camera_offset\" is expected to have 3 elements, got %ld", ros::this_node::getName().c_str(), camera_offset_raw.size());
    ros::shutdown();
  }
  camera_offset = Eigen::Vector3d(camera_offset_raw[0], camera_offset_raw[1], camera_offset_raw[2]);

  //}

  status = IDLE;
  tf_buffer_.reset(new tf2_ros::Buffer);
  tf2_ros::TransformListener tl(*tf_buffer_);

  for (int i = 0; i < DOF; i++) {
    joint_angles.push_back(0);
  }

  // action clients
  action_client_goto_.reset(new actionlib::SimpleActionClient<pouring_msgs::MoveGroupAction>(nh, "execute_trajectory", false));

  // service servers
  service_server_homing               = nh.advertiseService("home_in", &callbackHomingService);
  service_server_align_arm            = nh.advertiseService("align_arm_in", &callbackAlignArmService);
  service_server_goto                 = nh.advertiseService("goto_in", &callbackGoToService);
  service_server_goto_relative        = nh.advertiseService("goto_relative_in", &callbackGoToRelativeService);
  service_server_goto_angles          = nh.advertiseService("goto_angles_in", &callbackGoToAnglesService);
  service_server_goto_angles_relative = nh.advertiseService("goto_angles_relative_in", &callbackGoToAnglesRelativeService);
  service_server_goto_storage         = nh.advertiseService("goto_storage_in", &callbackGoToStorageService);
  service_server_lift_brick           = nh.advertiseService("lift_brick_in", &callbackLiftBrickService);
  service_server_pickup_brick         = nh.advertiseService("pickup_in", &callbackPickupBrickService);
  service_server_prepare_gripping     = nh.advertiseService("prepare_gripping_in", &callbackPrepareGrippingService);
  service_server_raise_camera         = nh.advertiseService("raise_camera_in", &callbackRaiseCameraService);
  service_server_store_brick          = nh.advertiseService("store_brick_in", &callbackStoreBrickService);
  service_server_unload_brick         = nh.advertiseService("unload_brick_in", &callbackUnloadBrickService);

  // service clients
  service_client_grip   = nh.serviceClient<std_srvs::Trigger>("grip_out");
  service_client_ungrip = nh.serviceClient<std_srvs::Trigger>("ungrip_out");

  // subscribers
  subscriber_joint_state         = nh.subscribe("joint_states_in", 1, &callbackJointStateTopic, ros::TransportHints().tcpNoDelay());
  subscriber_brick_pose          = nh.subscribe("brick_pose_in", 1, &callbackBrickPoseTopic, ros::TransportHints().tcpNoDelay());
  subscriber_gripper_diagnostics = nh.subscribe("gripper_diagnostics_in", 1, &callbackGripperDiagnosticsTopic, ros::TransportHints().tcpNoDelay());

  // publishers
  publisher_arm_status         = nh.advertise<mbzirc_husky_msgs::Gen3ArmStatus>("arm_status_out", 1);
  publisher_camera_to_ground   = nh.advertise<std_msgs::Float64>("camera_to_ground_out", 1);
  publisher_cartesian_velocity = nh.advertise<kortex_driver::TwistCommand>("cartesian_velocity_out", 1);
  publisher_rviz_markers       = nh.advertise<visualization_msgs::Marker>("markers_out", 1);

  ROS_INFO("[%s]: Waiting for arm feedback...", ros::this_node::getName().c_str());
  while (ros::ok() && !getting_joint_angles && !getting_end_effector_pose) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  status_timer = nh.createTimer(ros::Duration(1.0 / status_timer_rate), statusTimer);

  ROS_INFO("[%s]: Initialized! The arm manager is now ready to use", ros::this_node::getName().c_str());
  is_initialized = true;

  ros::spin();

  return 0;
}
//}
