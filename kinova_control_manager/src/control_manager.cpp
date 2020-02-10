#include <random>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>

#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseAction.h>

#include <mbzirc_husky_msgs/ArmStatus.h>
#include <mbzirc_husky_msgs/EndEffectorPose.h>
#include <mbzirc_husky_msgs/Vector3.h>
#include <mbzirc_husky_msgs/Float64.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>
#include <mbzirc_husky_msgs/StoragePosition.h>

#include <mrs_msgs/GripperDiagnostics.h>
#include <tf2_ros/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>

#define DOF 6
#define ALLOWED_NUM_OF_ERRORS 200

namespace kinova_control_manager
{

typedef enum
{
  MOVING,
  IDLE,
  HOMING,
  ALIGNING,
  PICKING,
} MotionStatus_t;

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
std::string statusToString(MotionStatus_t ms) {
  switch (ms) {
    case IDLE:
      return "IDLE";
    case MOVING:
      return "MOVING";
    case HOMING:
      return "HOMING";
    case ALIGNING:
      return "ALIGNING";
    case PICKING:
      return "PICKING";
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

/* class definition //{ */
class kinova_control_manager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized              = false;
  bool brick_attached              = false;
  bool gripper_engaged             = false;
  bool getting_joint_angles        = false;
  bool getting_tool_pose           = false;
  bool getting_gripper_diagnostics = false;

  bool getting_realsense_brick = false;
  bool brick_reliable          = false;

  ros::Time last_brick_time;
  ros::Time gripper_start_time;

  std::string arm_type;
  std::string husky_base_frame_id;
  std::string kinova_base_frame_id;

  // continuous status publishing
  ros::Timer status_timer;
  int        status_timer_rate;
  void       statusTimer(const ros::TimerEvent &evt);

  // position control
  bool goTo(Pose3d pose);
  void goToNonBlocking(Pose3d pose);
  bool goToRelative(Pose3d pose);

  // configuration params
  Pose3d home_pose;
  Pose3d default_gripping_pose;
  Pose3d raised_camera_pose;
  Pose3d default_firefighting_pose;
  Pose3d wrist_offset;

  // pose of the end effector when reaching for the N-th brick
  std::vector<Pose3d> storage_pose;
  std::vector<Pose3d> waypoints;

  double nearby_position_threshold;
  double nearby_rotation_threshold;
  double no_move_error_timeout;
  double gripper_timeout;
  double arm_base_to_ground;
  /* double linear_vel_modifier; */
  double move_down_speed_faster;
  double move_down_speed_slower;
  double move_down_speed_mega_slow;

  // arm status
  MotionStatus_t status;
  double         joint_angles[DOF];
  double         last_joint_angles[DOF];
  Pose3d         end_effector_pose_raw;
  Pose3d         end_effector_pose_compensated;
  Pose3d         last_goal;
  ros::Time      time_of_last_motion;
  Brick          detected_brick;

  // advertised services
  ros::ServiceServer service_server_homing;
  ros::ServiceServer service_server_prepare_gripping;
  ros::ServiceServer service_server_lift_brick;
  ros::ServiceServer service_server_raise_camera;
  ros::ServiceServer service_server_align_arm;
  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_relative;
  ros::ServiceServer service_server_pickup_brick;
  ros::ServiceServer service_server_goto_storage;
  ros::ServiceServer service_server_store_brick;
  ros::ServiceServer service_server_unload_brick;

  // called services
  ros::ServiceClient service_client_homing;
  // ros::ServiceClient service_client_brick_detector;
  ros::ServiceClient service_client_grip;
  ros::ServiceClient service_client_ungrip;

  // publishers
  ros::Publisher publisher_arm_status;
  ros::Publisher publisher_camera_to_ground;
  ros::Publisher publisher_end_effector_pose;
  ros::Publisher publisher_cartesian_velocity;
  ros::Publisher publisher_rviz_markers;

  // subscribers
  ros::Subscriber subscriber_joint_angles;
  ros::Subscriber subscriber_tool_pose;
  ros::Subscriber subscriber_brick_pose;
  ros::Subscriber subscriber_gripper_diagnostics;

  // service callbacks
  bool callbackHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackPrepareGrippingService(mbzirc_husky_msgs::Float64::Request &req, mbzirc_husky_msgs::Float64::Response &res);
  bool callbackLiftBrickService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res);
  bool callbackRaiseCameraService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackAlignArmService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res);
  bool callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);
  bool callbackGoToRelativeService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);
  bool callbackPickupBrickService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGoToStorageService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res);
  bool callbackStoreBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res);
  bool callbackUnloadBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res);

  // topic callbacks
  void callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg);
  void callbackToolPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackBrickPoseTopic(const mbzirc_husky_msgs::brickPositionConstPtr &msg);
  void callbackGripperDiagnosticsTopic(const mrs_msgs::GripperDiagnosticsConstPtr &msg);

  // gripper controls
  bool grip();
  bool ungrip();

  // utils
  bool nearbyAngleDeg(double a, double b);
  bool nearbyAngleRad(double a, double b);
  bool nearbyPose(Pose3d p, Pose3d q);
  bool nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v);

  // transform stuff
  tf2_ros::TransformBroadcaster tb;

  void publishTF();
  void flushVelocityTopic(kinova_msgs::PoseVelocity msg);

  int picking_error_counter = 0;

};
//}

/* onInit (constructor equivalent for nodelets) //{ */
void kinova_control_manager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  ROS_INFO("[kinova_control_manager]: Initializing...");

  // param containers
  std::vector<double> home_pose_raw;
  std::vector<double> gripping_pose_raw;
  std::vector<double> raised_camera_pose_raw;
  std::vector<double> firefighting_pose_raw;
  std::vector<double> husky_to_arm_base_transform;
  std::vector<double> wrist_offset_raw;
  std::vector<double> brick_storage_raw;
  std::vector<double> waypoints_raw;

  /* load params //{ */
  nh_.getParam("arm_type", arm_type);
  nh_.getParam("status_timer_rate", status_timer_rate);
  nh_.getParam("no_move_error_timeout", no_move_error_timeout);
  nh_.getParam("gripper_timeout", gripper_timeout);
  nh_.getParam("nearby_position_threshold", nearby_position_threshold);
  nh_.getParam("nearby_rotation_threshold", nearby_rotation_threshold);

  nh_.getParam("end_effector_home_pose", home_pose_raw);
  nh_.getParam("default_gripping_pose", gripping_pose_raw);
  nh_.getParam("raised_camera_pose", raised_camera_pose_raw);
  nh_.getParam("default_firefighting_pose", firefighting_pose_raw);
  nh_.getParam("husky_to_arm_base_transform", husky_to_arm_base_transform);
  nh_.getParam("wrist_offset", wrist_offset_raw);

  nh_.getParam("husky_base_frame_id", husky_base_frame_id);
  nh_.getParam("kinova_base_frame_id", kinova_base_frame_id);
  nh_.getParam("arm_base_to_ground", arm_base_to_ground);
  /* nh_.getParam("linear_vel_modifier", linear_vel_modifier); */
  nh_.getParam("move_down_speed_faster", move_down_speed_faster);
  nh_.getParam("move_down_speed_slower", move_down_speed_slower);
  nh_.getParam("move_down_speed_mega_slow", move_down_speed_mega_slow);

  nh_.getParam("brick_storage", brick_storage_raw);
  nh_.getParam("waypoints", waypoints_raw);
  //}

  /* parse params //{ */
  if (arm_type.size() < 4) {
    ROS_ERROR("[kinova_control_manager]: ARM_TYPE expected to be at least 4 characters!");
    ros::shutdown();
  }

  int dof = arm_type[3] - '0';
  if (dof != DOF) {
    ROS_ERROR("[kinova_control_manager]: Expected 6 DOF, got %d!", dof);
    ros::shutdown();
  }

  if (home_pose_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"end_effector_home\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!", home_pose_raw.size());
    ros::shutdown();
  }

  home_pose.pos = Eigen::Vector3d(home_pose_raw[0], home_pose_raw[1], home_pose_raw[2]);
  home_pose.rot = eulerToQuaternion(Eigen::Vector3d(home_pose_raw[3], home_pose_raw[4], home_pose_raw[5]));

  if (gripping_pose_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"default_gripping_pose\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!",
              gripping_pose_raw.size());
    ros::shutdown();
  }

  default_gripping_pose.pos = Eigen::Vector3d(gripping_pose_raw[0], gripping_pose_raw[1], gripping_pose_raw[2]);
  default_gripping_pose.rot = eulerToQuaternion(Eigen::Vector3d(gripping_pose_raw[3], gripping_pose_raw[4], gripping_pose_raw[5]));

  if (raised_camera_pose_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"raised_camera_pose\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!",
              raised_camera_pose_raw.size());
    ros::shutdown();
  }

  raised_camera_pose.pos = Eigen::Vector3d(raised_camera_pose_raw[0], raised_camera_pose_raw[1], raised_camera_pose_raw[2]);
  raised_camera_pose.rot = eulerToQuaternion(Eigen::Vector3d(raised_camera_pose_raw[3], raised_camera_pose_raw[4], raised_camera_pose_raw[5]));

  if (firefighting_pose_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"default_firefighting_pose\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!",
              firefighting_pose_raw.size());
    ros::shutdown();
  }

  default_firefighting_pose.pos = Eigen::Vector3d(firefighting_pose_raw[0], firefighting_pose_raw[1], firefighting_pose_raw[2]);
  default_firefighting_pose.rot = eulerToQuaternion(Eigen::Vector3d(firefighting_pose_raw[3], firefighting_pose_raw[4], firefighting_pose_raw[5]));

  if (wrist_offset_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"wrist_offset\" expected to have 3 elements [X,Y,Z,roll,pitch,yaw], got %ld!", wrist_offset_raw.size());
    ros::shutdown();
  }

  wrist_offset.pos = Eigen::Vector3d(wrist_offset_raw[0], wrist_offset_raw[1], wrist_offset_raw[2]);
  wrist_offset.rot = eulerToQuaternion(Eigen::Vector3d(wrist_offset_raw[3], wrist_offset_raw[4], wrist_offset_raw[5]));

  if (brick_storage_raw.size() % 6 == 0) {
    int num_storage_bins = (int)(brick_storage_raw.size() / 6);
    for (int i = 0; i < num_storage_bins; i++) {
      Pose3d pose;
      pose.pos.x() = brick_storage_raw[i * 6];
      pose.pos.y() = brick_storage_raw[(i * 6) + 1];
      pose.pos.z() = brick_storage_raw[(i * 6) + 2];
      Eigen::Vector3d angles(brick_storage_raw[(i * 6) + 3], brick_storage_raw[(i * 6) + 4], brick_storage_raw[(i * 6) + 5]);
      pose.rot = eulerToQuaternion(angles);
      storage_pose.push_back(pose);
    }
  }
  if (waypoints_raw.size() % 6 == 0) {
    int num_waypoints = (int)(waypoints_raw.size() / 6);
    for (int i = 0; i < num_waypoints; i++) {
      Pose3d pose;
      pose.pos.x() = waypoints_raw[i * 6];
      pose.pos.y() = waypoints_raw[(i * 6) + 1];
      pose.pos.z() = waypoints_raw[(i * 6) + 2];
      Eigen::Vector3d angles(waypoints_raw[(i * 6) + 3], waypoints_raw[(i * 6) + 4], waypoints_raw[(i * 6) + 5]);
      pose.rot = eulerToQuaternion(angles);
      waypoints.push_back(pose);
    }
  }
  //}

  // service servers
  service_server_homing           = nh_.advertiseService("home_in", &kinova_control_manager::callbackHomingService, this);
  service_server_prepare_gripping = nh_.advertiseService("prepare_gripping_in", &kinova_control_manager::callbackPrepareGrippingService, this);
  service_server_lift_brick       = nh_.advertiseService("lift_brick_in", &kinova_control_manager::callbackLiftBrickService, this);
  service_server_raise_camera     = nh_.advertiseService("raise_camera_in", &kinova_control_manager::callbackRaiseCameraService, this);
  service_server_align_arm        = nh_.advertiseService("align_arm_in", &kinova_control_manager::callbackAlignArmService, this);
  service_server_goto             = nh_.advertiseService("goto_in", &kinova_control_manager::callbackGoToService, this);
  service_server_goto_relative    = nh_.advertiseService("goto_relative_in", &kinova_control_manager::callbackGoToRelativeService, this);
  service_server_goto_storage     = nh_.advertiseService("goto_storage_in", &kinova_control_manager::callbackGoToStorageService, this);
  service_server_store_brick      = nh_.advertiseService("store_brick_in", &kinova_control_manager::callbackStoreBrickService, this);
  service_server_unload_brick     = nh_.advertiseService("unload_brick_in", &kinova_control_manager::callbackUnloadBrickService, this);
  service_server_pickup_brick     = nh_.advertiseService("pickup_in", &kinova_control_manager::callbackPickupBrickService, this);

  // service clients
  service_client_homing = nh_.serviceClient<kinova_msgs::HomeArm>("home_out");
  service_client_grip   = nh_.serviceClient<std_srvs::Trigger>("grip_out");
  service_client_ungrip = nh_.serviceClient<std_srvs::Trigger>("ungrip_out");

  // publishers
  publisher_arm_status         = nh_.advertise<mbzirc_husky_msgs::ArmStatus>("arm_status_out", 1);
  publisher_camera_to_ground   = nh_.advertise<std_msgs::Float64>("camera_to_ground_out", 1);
  publisher_end_effector_pose  = nh_.advertise<kinova_msgs::ArmPoseActionGoal>("end_effector_pose_out", 1);
  publisher_cartesian_velocity = nh_.advertise<kinova_msgs::PoseVelocity>("cartesian_velocity_out", 1);
  publisher_rviz_markers       = nh_.advertise<visualization_msgs::Marker>("markers_out", 1);

  // subscribers
  subscriber_joint_angles = nh_.subscribe("joint_angles_in", 1, &kinova_control_manager::callbackJointAnglesTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_brick_pose   = nh_.subscribe("brick_pose_in", 1, &kinova_control_manager::callbackBrickPoseTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_gripper_diagnostics =
      nh_.subscribe("gripper_diagnostics_in", 1, &kinova_control_manager::callbackGripperDiagnosticsTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_tool_pose = nh_.subscribe("tool_pose_in", 1, &kinova_control_manager::callbackToolPoseTopic, this, ros::TransportHints().tcpNoDelay());

  // timers
  status_timer = nh_.createTimer(ros::Rate(status_timer_rate), &kinova_control_manager::statusTimer, this);

  status = IDLE;

  for (int i = 0; i < DOF; i++) {
    joint_angles[i]      = 0.0;
    last_joint_angles[i] = 0.0;
  }

  ROS_INFO("[kinova_control_manager]: Waiting for arm feedback...");

  while (!getting_joint_angles || !getting_tool_pose) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  last_goal = end_effector_pose_compensated;

  is_initialized = true;
  ROS_INFO("[kinova_control_manager]: Initialized Kinova Arm type: %s", arm_type.c_str());

}  // namespace kinova_control_manager
//}

/* callbackHomingService //{ */
bool kinova_control_manager::callbackHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Reset last arm goal. Homing...");
  
  status              = HOMING;
  time_of_last_motion = ros::Time::now();
  last_goal           = home_pose;

  ungrip();

  kinova_msgs::HomeArm msg;
  service_client_homing.call(msg.request, msg.response);
  while (status != IDLE) {
    ros::spinOnce();
    ros::Duration(0.05).sleep();
  }
  res.success = true;
  return true;
}
//}

/* callbackPrepareGrippingService //{ */
// input value is offset in Z axis relative to the default gripping position
bool kinova_control_manager::callbackPrepareGrippingService(mbzirc_husky_msgs::Float64::Request &req, mbzirc_husky_msgs::Float64::Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    Pose3d goal_pose = default_gripping_pose;
    goal_pose.pos.z() += req.data;

    if (nearbyPose(end_effector_pose_compensated, goal_pose)) {
      ROS_INFO("[kinova_control_manager]: Assumed a default gripping pose");
      status      = IDLE;
      res.success = true;
      return true;
    }

    ROS_ERROR("[kinova_control_manager]: Cannot move, arm is not IDLE");
    res.success = false;
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Assuming a default gripping pose");
  status              = MotionStatus_t::MOVING;
  time_of_last_motion = ros::Time::now();
  last_goal           = default_gripping_pose;

  Pose3d goal_pose = default_gripping_pose;
  goal_pose.pos.z() += req.data;
  last_goal = goal_pose;

  // bool goal_reached = goTo(goal_pose);
  goToNonBlocking(goal_pose);

  res.success = true;
  return true;
}
//}

/* callbackLiftBrickService //{ */
// input value is offset in Z axis relative to the default gripping position
bool kinova_control_manager::callbackLiftBrickService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, arm is not IDLE!");
    res.success = false;
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Assuming a default gripping pose");
  status              = MotionStatus_t::MOVING;
  time_of_last_motion = ros::Time::now();
  last_goal           = default_gripping_pose;

  Pose3d goal_pose = default_gripping_pose;
  goal_pose.pos.z() += req.data;
  last_goal = goal_pose;

  bool have_brick   = brick_attached;
  bool goal_reached = goTo(goal_pose);

  if (have_brick != brick_attached) {
    ROS_ERROR("[kinova_control_manager]: Brick lost during ascent!");
    res.success = false;
    return false;
  }

  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackRaiseCameraService //{ */
bool kinova_control_manager::callbackRaiseCameraService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, arm is not IDLE!");
    res.success = false;
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Assuming a raised camera pose");
  status              = MotionStatus_t::MOVING;
  time_of_last_motion = ros::Time::now();
  last_goal           = raised_camera_pose;
  goTo(raised_camera_pose);

  while (status != MotionStatus_t::IDLE) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  res.success = true;
  return true;
}
//}

/* callbackAlignArmService //{ */
bool kinova_control_manager::callbackAlignArmService(mbzirc_husky_msgs::Float64Request &req, mbzirc_husky_msgs::Float64Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot align_arm, not initialized!");
    res.success = false;
    res.message = "Cannot align_arm, not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot align arm, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot align arm, arm is not IDLE!");
    res.success = false;
    return false;
  }

  status = ALIGNING;

  if (!getting_realsense_brick) {
    ROS_FATAL("[kinova_arm_manager]: Brick pose topic did not open!");
    status      = IDLE;
    res.success = false;
    return false;
  }

  bool aligned = false;

  for (int i = 0; i < 10; i++) {

    if (aligned) {
      break;
    }
    if (!getting_realsense_brick) {
      status      = IDLE;
      res.success = false;
      return false;
    }

    Eigen::Vector3d brick_euler = quaternionToEuler(detected_brick.pose.rot);

    Eigen::Vector3d align       = Eigen::Vector3d(-detected_brick.pose.pos.x(), detected_brick.pose.pos.y(), brick_euler.z());
    Eigen::Vector3d align_euler = quaternionToEuler(default_gripping_pose.rot);
    Eigen::Vector3d ee_euler    = quaternionToEuler(end_effector_pose_raw.rot);
    align_euler.z()             = ee_euler.z() + brick_euler.z() + M_PI;

    ROS_INFO("[kinova_control_manager]: Brick yaw: %.2f", brick_euler.z());
    ROS_INFO("[kinova_control_manager]: Suggested alignment: [%.2f, %.2f, %.2f]", align[0], align[1], align[2]);

    Pose3d new_pose;
    new_pose.pos.x() = end_effector_pose_raw.pos.x() + align.x();
    new_pose.pos.y() = end_effector_pose_raw.pos.y() + align.y();
    new_pose.pos.z() = default_gripping_pose.pos.z() + req.data;
    new_pose.rot     = eulerToQuaternion(align_euler) * wrist_offset.rot.inverse();
    goToNonBlocking(new_pose);
    ros::Duration(0.7).sleep();

    aligned = (align.x() < 0.05 && align.y() < 0.05 && align.z() < 0.05);
  }
  if (end_effector_pose_compensated.pos.y() > -0.41 || end_effector_pose_compensated.pos.y() < -0.59) {
    ROS_ERROR("[kinova_control_manager]: Alignment in Y axis failed.");
    status = IDLE;
    ros::spinOnce();
    ros::Duration(1.5).sleep();  // try removing this sleep
    ros::spinOnce();
    res.success = false;
    return false;
  }
  status = IDLE;
  ros::spinOnce();
  ros::Duration(1.5).sleep();  // try removing this sleep
  ros::spinOnce();
  res.success = aligned;
  return aligned;
}
//}

/* callbackPickupBrickService //{ */
bool kinova_control_manager::callbackPickupBrickService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot start \"pickup brick\", arm is not IDLE!");
    res.success = false;
    return false;
  }

  if (!getting_realsense_brick && !getting_gripper_diagnostics) {
    ROS_WARN("[kinova_control_manager]: Cannot pickup brick whithout realsense and gripper feedback!");
    res.success = false;
    return false;
  }

  picking_error_counter = 0;
  status              = PICKING;
  time_of_last_motion = ros::Time::now();

  kinova_msgs::PoseVelocity msg;
  ROS_INFO("[kinova_control_manager]: Moving down");

  double stopping_height = (detected_brick.brick_layer * 0.2) - arm_base_to_ground + 0.07;
  std::cout << "Slow down at Z: " << stopping_height << "\n";

  while (!brick_attached && brick_reliable) {

	  if(picking_error_counter > ALLOWED_NUM_OF_ERRORS){
	  	ROS_ERROR("[kinova_control_manager]: Arm is probably stuck. Consider homing");
		flushVelocityTopic(msg);
		res.success = false;
		return false;
	  }

    if (!getting_realsense_brick) {
      ROS_ERROR("[kinova_control_manager]: Brick lost, aborting pickup");
      flushVelocityTopic(msg);
      status      = IDLE;
      res.success = false;
      return false;
    }
    if ((ros::Time::now() - time_of_last_motion).toSec() > no_move_error_timeout) {
      ROS_ERROR("[kinova_control_manager]: Arm is not moving! Pickup failed");
      flushVelocityTopic(msg);
      status      = IDLE;
      res.success = false;
      return false;
    }

    msg.twist_linear_x = -detected_brick.pose.pos.x();
    msg.twist_linear_y = detected_brick.pose.pos.y();
    msg.twist_linear_z = -move_down_speed_faster;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.01).sleep();
  }

  // now the descent will not abort if brick is lost
  ROS_WARN("[kinova_control_manager]: Entering the danger zone!");

  while (!brick_attached && end_effector_pose_compensated.pos.z() > stopping_height) {

    if ((ros::Time::now() - time_of_last_motion).toSec() > no_move_error_timeout) {
      ROS_ERROR("[kinova_control_manager]: Arm is not moving! Pickup failed. Homing may be required");
      flushVelocityTopic(msg);
      status      = IDLE;
      res.success = false;
      return false;
    }

    msg.twist_linear_x  = 0.0;
    msg.twist_linear_y  = 0.0;
    msg.twist_linear_z  = -move_down_speed_slower;
    msg.twist_angular_x = 0.0;
    msg.twist_angular_y = 0.0;
    msg.twist_angular_z = 0.0;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.01).sleep();
  }

  grip();
  ROS_WARN("[kinova_control_manager]:MEGA slow now");

  while (!brick_attached) {
    if ((end_effector_pose_compensated.pos.z() + arm_base_to_ground) < ((detected_brick.brick_layer * 0.2) - 0.06)) {
      // if it smashes into the bricks too hard, increase the last number in the condition
      ROS_FATAL("[kinova_control_manager]: Failed to attach the brick!");
      flushVelocityTopic(msg);
      publisher_cartesian_velocity.publish(msg);
      status = IDLE;
      ungrip();
      res.success = false;
      return false;
    }
    msg.twist_linear_x  = 0.0;
    msg.twist_linear_y  = 0.0;
    msg.twist_linear_z  = -move_down_speed_mega_slow;
    msg.twist_angular_x = 0.0;
    msg.twist_angular_y = 0.0;
    msg.twist_angular_z = 0.0;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.009).sleep();
  }

  flushVelocityTopic(msg);
  publisher_cartesian_velocity.publish(msg);
  status      = IDLE;
  res.success = true;
  return true;
}
//}

/* callbackGoToService //{ */
bool kinova_control_manager::callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res) {

  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goTo\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goTo\", not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goTo\", internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goTo\", arm is not IDLE!");
    res.success = false;
    res.message = "Cannot execute \"goTo\", arm is not IDLE!";
    return false;
  }
  status              = MOVING;
  time_of_last_motion = ros::Time::now();

  Pose3d pose;
  pose.pos          = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot          = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  bool goal_reached = goTo(pose);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToRelativeService //{ */
bool kinova_control_manager::callbackGoToRelativeService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res) {

  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToRelative\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToRelative\", internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToRelative\", arm is not IDLE!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", arm is not IDLE!";
    return false;
  }
  status              = MOVING;
  time_of_last_motion = ros::Time::now();

  Pose3d pose;
  pose.pos          = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot          = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  bool goal_reached = goToRelative(pose);
  res.success       = goal_reached;
  return goal_reached;
}
//}

/* callbackGoToStorageService //{ */
bool kinova_control_manager::callbackGoToStorageService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {

  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToStorage\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goToStorage\", not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToStorage\", internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToStorage\", arm is not IDLE!");
    res.success = false;
    res.message = "Cannot execute \"goToStorage\", arm is not IDLE!";
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Moving arm to storage position %d, layer %d", req.position, req.layer);
  bool have_brick     = brick_attached;
  status              = MOVING;
  time_of_last_motion = ros::Time::now();

  // this is some high level collision avoidance stuff
  if(req.num_of_waypoints > 0 && req.num_of_waypoints < 3){
  	for(int i = 0; i < req.num_of_waypoints; i++){
  		ROS_INFO("[kinova_control_manager]: Performing collision avoidance. Going to waypoint %d", i);
  		Pose3d waypoint = waypoints[i];
  		goTo(waypoint);
  	}
  	ROS_INFO("[kinova_control_manager]: All waypoints reached, returning to original goal");
  }
  
  status              = MOVING;
  time_of_last_motion = ros::Time::now();
  Pose3d pose = storage_pose[req.position];
  pose.pos.z() += 0.2 * req.layer;
  bool goal_reached = goTo(pose);

  if (have_brick != brick_attached) {
    ROS_ERROR("[kinova_control_manager]: Brick lost during motion!");
    res.success = false;
    return false;
  }

  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackStoreBrickService //{ */
bool kinova_control_manager::callbackStoreBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {
  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot store brick, not initialized!");
    res.success = false;
    res.message = "Cannot execute deploy brick into storage, not initialized!";
    return false;
  }

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot store brick, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (status != IDLE) {
    ROS_ERROR("[kinova_control_manager]: Cannot store brick, arm is not IDLE!");
    res.success = false;
    res.message = "Cannot store brick, arm is not IDLE!";
    return false;
  }

  status              = MOVING;
  time_of_last_motion = ros::Time::now();

  Pose3d new_goal = storage_pose[req.position];
  new_goal.pos.z() += (0.2 * req.layer) - 0.3;
  bool goal_reached = goTo(new_goal);
  ungrip();
  ros::Duration(0.2).sleep();
  res.success = goal_reached;
  return goal_reached;
}
//}

/* callbackUnloadBrickService //{ */
bool kinova_control_manager::callbackUnloadBrickService(mbzirc_husky_msgs::StoragePosition::Request &req, mbzirc_husky_msgs::StoragePosition::Response &res) {
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

  status              = MOVING;
  time_of_last_motion = ros::Time::now();

  Pose3d new_goal = storage_pose[req.position];
  new_goal.pos.z() += (0.2 * req.layer) - 0.25;
  goTo(new_goal);
  grip();
  new_goal.pos.z() -= 0.07;
  goTo(new_goal);

  if (!brick_attached) {
    ROS_ERROR("[kinova_control_manager]: Failed to attach brick!");
    status      = IDLE;
    res.success = false;
    return false;
  }
  status      = IDLE;
  res.success = true;
  return true;
}
//}

/* callbackJointAnglesTopic //{ */
void kinova_control_manager::callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg) {
  getting_joint_angles = true;

  for (int i = 0; i < DOF; i++) {
    last_joint_angles[i] = joint_angles[i];
  }

  joint_angles[0] = msg->joint1;
  joint_angles[1] = msg->joint2;
  joint_angles[2] = msg->joint3;
  joint_angles[3] = msg->joint4;
  joint_angles[4] = msg->joint5;
  joint_angles[5] = msg->joint6;

  for (int i = 0; i < DOF; i++) {
    if (!nearbyAngleDeg(joint_angles[i], last_joint_angles[i])) {
      time_of_last_motion = ros::Time::now();
      return;
    }
  }
  if (status == PICKING){
  	picking_error_counter++;
  }

  /* idle handler //{ */
  if (status == IDLE) {
    return;
  }
  //}

  if ((ros::Time::now() - time_of_last_motion).toSec() > no_move_error_timeout) {
    /* homing handler //{ */
    if (status == HOMING) {
      if (nearbyPose(home_pose, end_effector_pose_compensated)) {
        ROS_INFO("[Arm manager]: Homing complete");
      } else {
        ROS_ERROR("[Arm manager]: Homing error! Check arm collisions");
      }
      status = IDLE;
      return;
    }
    //}

    /* moving handler //{ */
    if (status == MOVING) {
      if (nearbyPose(last_goal, end_effector_pose_compensated)) {
        ROS_INFO("[Arm manager]: Goal reached");
      } else {
        Eigen::Vector3d p_euler = quaternionToEuler(last_goal.rot);
        Eigen::Vector3d q_euler = quaternionToEuler(end_effector_pose_compensated.rot);
        Eigen::Vector3d angular_diff;

        for (int i = 0; i < 3; i++) {
          angular_diff[i] = std::abs(p_euler[i] - q_euler[i]);
          while (angular_diff[i] > 3.0) {
            angular_diff[i] -= 3.0;
          }
          while (angular_diff[i] < -3.0) {
            angular_diff[i] += 3.0;
          }
        }
        std::cout << "angular diff: " << angular_diff << "\n";
        double pos_error = (last_goal.pos - end_effector_pose_compensated.pos).norm();
        ROS_WARN("[Arm manager]: Destination unreachable. Position error: %.4f, Rotation error: %.2f, %.2f, %.2f", pos_error, angular_diff[0], angular_diff[1],
                 angular_diff[2]);
      }
      status = IDLE;
    }
    //}
  }
}
//}

/* callbackBrickPoseTopic //{ */
void kinova_control_manager::callbackBrickPoseTopic(const mbzirc_husky_msgs::brickPositionConstPtr &msg) {
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

    double end_effector_to_ground = end_effector_pose_compensated.pos.z() + arm_base_to_ground;
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

/* callbackGripperDiagnosticsTopic //{ */
void kinova_control_manager::callbackGripperDiagnosticsTopic(const mrs_msgs::GripperDiagnosticsConstPtr &msg) {
  getting_gripper_diagnostics = true;
  if (!gripper_engaged && msg->gripper_on) {
    gripper_start_time = ros::Time::now();
  }
  gripper_engaged = msg->gripper_on;
  brick_attached  = msg->gripping_object;
}
//}

/* statusTimer //{ */
void kinova_control_manager::statusTimer([[maybe_unused]] const ros::TimerEvent &evt) {

  if (!is_initialized) {
    return;
  }
  mbzirc_husky_msgs::ArmStatus status_msg;
  status_msg.arm_type     = arm_type;
  status_msg.header.stamp = ros::Time::now();
  std::stringstream ss;
  ss << arm_type << "_link_base";
  status_msg.header.frame_id = ss.str().c_str();
  status_msg.status          = statusToString(status);
  for (int i = 0; i < DOF; i++) {
    status_msg.joint_angles[i] = joint_angles[i];
  }
  status_msg.gripper_engaged = gripper_engaged;

  if (gripper_engaged && !brick_attached && (ros::Time::now() - gripper_start_time).toSec() > gripper_timeout) {
    ROS_WARN("[kinova_control_manager]: Gripper timeout! Turning the gripper off!");
    ungrip();
  }

  Eigen::Vector3d euler      = quaternionToEuler(end_effector_pose_compensated.rot);
  Eigen::Vector3d last_euler = quaternionToEuler(last_goal.rot);
  for (int i = 0; i < 3; i++) {
    status_msg.end_effector_pose[i]     = end_effector_pose_compensated.pos[i];
    status_msg.end_effector_pose[i + 3] = euler[i];
    status_msg.last_goal[i]             = last_goal.pos[i];
    status_msg.last_goal[i + 3]         = last_euler[i];
  }

  if (status == ALIGNING && (ros::Time::now() - last_brick_time).sec > 2.0) {
    ROS_WARN("[kinova_control_manager]: Brick timeout!");
    getting_realsense_brick     = false;
    detected_brick.pose.pos.x() = default_gripping_pose.pos.x();
    detected_brick.pose.pos.y() = default_gripping_pose.pos.y();
  }

  publisher_arm_status.publish(status_msg);

  std_msgs::Float64 cam_to_ground;
  cam_to_ground.data = end_effector_pose_compensated.pos.z() + arm_base_to_ground;
  publisher_camera_to_ground.publish(cam_to_ground);

  publishTF();
}
//}

/* goTo (blocking) //{ */
bool kinova_control_manager::goTo(Pose3d pose) {
  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[kinova_control_manager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(),
           euler[0], euler[1], euler[2]);

  last_goal = pose;  // store last_goal before compensation (this is the actual position of the wrist)

  // offset compensation because no wrist is attached
  pose.rot *= wrist_offset.rot;
  pose.pos += pose.rot * wrist_offset.pos;

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

  while (status != MotionStatus_t::IDLE) {
	if(brick_attached){
  		msg.goal.pose.pose.position.x = end_effector_pose_compensated.pos.x();
  		msg.goal.pose.pose.position.y = end_effector_pose_compensated.pos.y();
  		msg.goal.pose.pose.position.z = end_effector_pose_compensated.pos.z();

  		msg.goal.pose.pose.orientation.x = end_effector_pose_compensated.rot.x();
  		msg.goal.pose.pose.orientation.y = end_effector_pose_compensated.rot.y();
  		msg.goal.pose.pose.orientation.z = end_effector_pose_compensated.rot.z();
  		msg.goal.pose.pose.orientation.w = end_effector_pose_compensated.rot.w();

  		std::stringstream ss;
  		ss << arm_type << "_link_base";
  		msg.goal.pose.header.frame_id = ss.str().c_str();
  		msg.header.frame_id           = ss.str().c_str();

  		publisher_end_effector_pose.publish(msg);
		
		status = IDLE;
	  	return true;
	}
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  return nearbyPose(end_effector_pose_compensated, last_goal);
}
//}

/* goToNonBlocking //{ */
void kinova_control_manager::goToNonBlocking(Pose3d pose) {
  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[kinova_control_manager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(),
           euler[0], euler[1], euler[2]);

  last_goal = pose;  // store last_goal before compensation (this is the actual position of the wrist)

  // offset compensation because no wrist is attached
  pose.rot *= wrist_offset.rot;
  pose.pos += pose.rot * wrist_offset.pos;

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

  ros::spinOnce();
  ros::Duration(1.2).sleep();
  ros::spinOnce();
}
//}

/* goToRelative (blocking) //{ */
bool kinova_control_manager::goToRelative(Pose3d rel_pose) {

  Pose3d goal_pose  = last_goal + rel_pose;
  last_goal         = goal_pose;
  bool goal_reached = goTo(goal_pose);
  return goal_reached;
}
//}

/* grip //{ */
bool kinova_control_manager::grip() {
  std_srvs::Trigger trig;
  service_client_grip.call(trig.request, trig.response);
  ROS_INFO_STREAM("[kinova_control_manager]: " << trig.response.message);
  return trig.response.success;
}
//}

/* ungrip //{ */
bool kinova_control_manager::ungrip() {
  std_srvs::Trigger trig;
  service_client_ungrip.call(trig.request, trig.response);
  ROS_INFO_STREAM("[kinova_control_manager]: " << trig.response.message);
  return trig.response.success;
}
//}

/* nearbyAngleDeg //{ */
bool kinova_control_manager::nearbyAngleDeg(double a, double b) {
  return std::abs(a - b) < 0.2;
}
//}

/* nearbyAngleRad //{ */
bool kinova_control_manager::nearbyAngleRad(double a, double b) {
  return std::abs(b - a) < nearby_rotation_threshold;
}
//}

/* nearbyPose //{ */
bool kinova_control_manager::nearbyPose(Pose3d p, Pose3d q) {
  double pos_diff     = (p.pos - q.pos).norm();
  double angular_diff = p.rot.angularDistance(q.rot);

  std::cout << "Angular diff: " << angular_diff << "\n";

  return (pos_diff < nearby_position_threshold) && (angular_diff < nearby_rotation_threshold);
}
//}

/* nearbyVector //{ */
bool kinova_control_manager::nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v) {
  return (u - v).norm() < nearby_position_threshold;
}
//}

/* publishTF //{ */
void kinova_control_manager::publishTF() {
  if (!is_initialized || !getting_joint_angles) {
    return;
  }

  geometry_msgs::TransformStamped trans;
  trans.header.stamp    = ros::Time::now();
  trans.header.frame_id = husky_base_frame_id;
  trans.child_frame_id  = kinova_base_frame_id;
  /* trans.transform.translation.x = husky_to_arm_base_transform[0]; */
  /* trans.transform.translation.y = husky_to_arm_base_transform[1]; */
  /* trans.transform.translation.z = husky_to_arm_base_transform[2]; */
  trans.transform.translation.x = 0.0;
  trans.transform.translation.y = 0.0;
  /* trans.transform.translation.z = 0.0; */
  trans.transform.translation.z = arm_base_to_ground;
  Eigen::Quaterniond rot        = Eigen::Quaterniond::Identity();
  trans.transform.rotation.w    = rot.w();
  trans.transform.rotation.x    = rot.x();
  trans.transform.rotation.y    = rot.y();
  trans.transform.rotation.z    = rot.z();
  tb.sendTransform(trans);

  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = kinova_base_frame_id;
  trans.child_frame_id          = "end_effector_compensated";
  trans.transform.translation.x = end_effector_pose_compensated.pos.x();
  trans.transform.translation.y = end_effector_pose_compensated.pos.y();
  trans.transform.translation.z = end_effector_pose_compensated.pos.z();
  trans.transform.rotation.w    = end_effector_pose_compensated.rot.w();
  trans.transform.rotation.x    = end_effector_pose_compensated.rot.x();
  trans.transform.rotation.y    = end_effector_pose_compensated.rot.y();
  trans.transform.rotation.z    = end_effector_pose_compensated.rot.z();
  tb.sendTransform(trans);

  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = kinova_base_frame_id;
  trans.child_frame_id          = "end_effector_raw";
  trans.transform.translation.x = end_effector_pose_raw.pos.x();
  trans.transform.translation.y = end_effector_pose_raw.pos.y();
  trans.transform.translation.z = end_effector_pose_raw.pos.z();
  trans.transform.rotation.w    = end_effector_pose_raw.rot.w();
  trans.transform.rotation.x    = end_effector_pose_raw.rot.x();
  trans.transform.rotation.y    = end_effector_pose_raw.rot.y();
  trans.transform.rotation.z    = end_effector_pose_raw.rot.z();
  tb.sendTransform(trans);

  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = kinova_base_frame_id;
  trans.child_frame_id          = "last_goal";
  trans.transform.translation.x = last_goal.pos.x();
  trans.transform.translation.y = last_goal.pos.y();
  trans.transform.translation.z = last_goal.pos.z();
  trans.transform.rotation.w    = last_goal.rot.w();
  trans.transform.rotation.x    = last_goal.rot.x();
  trans.transform.rotation.y    = last_goal.rot.y();
  trans.transform.rotation.z    = last_goal.rot.z();
  tb.sendTransform(trans);

  if (brick_reliable) {
    visualization_msgs::Marker msg;
    msg.header.frame_id    = "end_effector_compensated";
    msg.header.stamp       = ros::Time::now();
    msg.ns                 = "current_namespace";
    msg.action             = visualization_msgs::Marker::ADD;
    msg.type               = visualization_msgs::Marker::CUBE;
    msg.id                 = 666;
    msg.pose.position.x    = detected_brick.pose.pos.x();
    msg.pose.position.y    = detected_brick.pose.pos.y();
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

/* callbackToolPoseTopic //{ */
void kinova_control_manager::callbackToolPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg) {
  getting_tool_pose             = true;
  end_effector_pose_raw.rot.w() = msg->pose.orientation.w;
  end_effector_pose_raw.rot.x() = msg->pose.orientation.x;
  end_effector_pose_raw.rot.y() = msg->pose.orientation.y;
  end_effector_pose_raw.rot.z() = msg->pose.orientation.z;

  end_effector_pose_raw.pos.x() = msg->pose.position.x;
  end_effector_pose_raw.pos.y() = msg->pose.position.y;
  end_effector_pose_raw.pos.z() = msg->pose.position.z;

  end_effector_pose_compensated.pos = end_effector_pose_raw.pos - (end_effector_pose_raw.rot * wrist_offset.pos);
  end_effector_pose_compensated.rot = wrist_offset.rot * end_effector_pose_raw.rot;
}
//}

/* flushVelocityTopic //{ */
void kinova_control_manager::flushVelocityTopic(kinova_msgs::PoseVelocity msg) {
  msg.twist_linear_x  = 0.0;
  msg.twist_linear_y  = 0.0;
  msg.twist_linear_z  = 0.0;
  msg.twist_angular_x = 0.0;
  msg.twist_angular_y = 0.0;
  msg.twist_angular_z = 0.0;
  publisher_cartesian_velocity.publish(msg);
  ros::spinOnce();
  ros::Duration(0.01).sleep();
  ros::spinOnce();
}
//}

/* dbgRandomNoise (UNUSED) //{ */
/* bool kinova_control_manager::dbgRandomNoise(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) { */
/*   std::cout << "dbg noise \n"; */
/*   kinova_msgs::PoseVelocity msg; */

/*   std::random_device               rd; */
/*   std::mt19937                     eng(rd()); */
/*   std::uniform_real_distribution<> dist(0, 0.1); */

/*   for (int i = 0; i < 400; i++) { */

/*     msg.twist_linear_x = 0.15 + dist(eng); */
/*     msg.twist_linear_y = -0.05 + dist(eng); */
/*     publisher_cartesian_velocity.publish(msg); */
/*     ros::Duration(0.01).sleep(); */
/*   } */
/*   res.success = true; */
/*   return true; */
/* } */
//}

}  // namespace kinova_control_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kinova_control_manager::kinova_control_manager, nodelet::Nodelet)
