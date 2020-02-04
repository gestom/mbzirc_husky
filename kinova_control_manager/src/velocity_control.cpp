#include <random>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>

#include <std_srvs/Trigger.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseAction.h>

#include <mbzirc_husky_msgs/ArmStatus.h>
#include <mbzirc_husky_msgs/EndEffectorPose.h>
#include <mbzirc_husky_msgs/Vector3.h>
#include <mbzirc_husky_msgs/brickDetect.h>
#include <mbzirc_husky_msgs/brickPosition.h>

#include <mrs_msgs/GripperDiagnostics.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <visualization_msgs/Marker.h>

#define DOF 6

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
  bool getting_joint_angles        = false;
  bool getting_gripper_diagnostics = false;

  bool getting_realsense_brick = false;
  bool brick_reliable          = false;

  ros::Time last_brick_time;

  std::string arm_type;
  std::string husky_base_frame_id;
  std::string kinova_base_frame_id;

  // continuous status publishing
  ros::Timer status_timer;
  int        status_timer_rate;
  void       statusTimer(const ros::TimerEvent &evt);

  // position control
  void goTo(Pose3d pose);
  void goToRelative(Pose3d pose);

  // configuration params
  Pose3d home_pose;
  Pose3d default_gripping_pose;
  Pose3d default_firefighting_pose;
  Pose3d wrist_offset;

  double nearby_position_threshold;
  double nearby_rotation_threshold;
  double no_move_error_timeout;
  double arm_base_to_ground;
  double linear_vel_modifier;
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
  ros::ServiceServer service_server_align_arm;
  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_relative;
  ros::ServiceServer service_server_pickup_brick;

  // called services
  ros::ServiceClient service_client_homing;
  ros::ServiceClient service_client_brick_detector;
  ros::ServiceClient service_client_grip;
  ros::ServiceClient service_client_ungrip;

  // publishers
  ros::Publisher publisher_arm_status;
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
  bool callbackPrepareGrippingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackAlignArmService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);
  bool callbackGoToRelativeService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);
  bool callbackPickupBrickService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

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
  std::vector<double> firefighting_pose_raw;
  std::vector<double> husky_to_arm_base_transform;
  std::vector<double> wrist_offset_raw;

  /* load params //{ */
  nh_.getParam("arm_type", arm_type);
  nh_.getParam("status_timer_rate", status_timer_rate);
  nh_.getParam("no_move_error_timeout", no_move_error_timeout);
  nh_.getParam("nearby_position_threshold", nearby_position_threshold);
  nh_.getParam("nearby_rotation_threshold", nearby_rotation_threshold);

  nh_.getParam("end_effector_home_pose", home_pose_raw);
  nh_.getParam("default_gripping_pose", gripping_pose_raw);
  nh_.getParam("default_firefighting_pose", firefighting_pose_raw);
  nh_.getParam("husky_to_arm_base_transform", husky_to_arm_base_transform);
  nh_.getParam("wrist_offset", wrist_offset_raw);

  nh_.getParam("husky_base_frame_id", husky_base_frame_id);
  nh_.getParam("kinova_base_frame_id", kinova_base_frame_id);
  nh_.getParam("arm_base_to_ground", arm_base_to_ground);
  nh_.getParam("linear_vel_modifier", linear_vel_modifier);
  nh_.getParam("move_down_speed_faster", move_down_speed_faster);
  nh_.getParam("move_down_speed_slower", move_down_speed_slower);
  nh_.getParam("move_down_speed_mega_slow", move_down_speed_mega_slow);
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
              home_pose_raw.size());
    ros::shutdown();
  }

  default_gripping_pose.pos = Eigen::Vector3d(gripping_pose_raw[0], gripping_pose_raw[1], gripping_pose_raw[2]);
  default_gripping_pose.rot = eulerToQuaternion(Eigen::Vector3d(gripping_pose_raw[3], gripping_pose_raw[4], gripping_pose_raw[5]));

  if (firefighting_pose_raw.size() != 6) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"default_firefighting_pose\" expected to have 6 elements [X,Y,Z,roll,pitch,yaw], got %ld!",
              home_pose_raw.size());
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

  //}

  // service servers
  service_server_homing           = nh_.advertiseService("home_in", &kinova_control_manager::callbackHomingService, this);
  service_server_prepare_gripping = nh_.advertiseService("prepare_gripping_in", &kinova_control_manager::callbackPrepareGrippingService, this);
  service_server_align_arm        = nh_.advertiseService("align_arm_in", &kinova_control_manager::callbackAlignArmService, this);
  service_server_goto             = nh_.advertiseService("goto_in", &kinova_control_manager::callbackGoToService, this);
  service_server_goto_relative    = nh_.advertiseService("goto_relative_in", &kinova_control_manager::callbackGoToRelativeService, this);
  service_server_pickup_brick     = nh_.advertiseService("pickup_in", &kinova_control_manager::callbackPickupBrickService, this);

  // service clients
  service_client_homing         = nh_.serviceClient<kinova_msgs::HomeArm>("home_out");
  service_client_brick_detector = nh_.serviceClient<mbzirc_husky_msgs::brickDetect>("brick_detect");
  service_client_grip           = nh_.serviceClient<std_srvs::Trigger>("grip_out");
  service_client_ungrip         = nh_.serviceClient<std_srvs::Trigger>("ungrip_out");

  // publishers
  publisher_arm_status         = nh_.advertise<mbzirc_husky_msgs::ArmStatus>("arm_status_out", 1);
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
  last_goal = home_pose;

  tf2_ros::Buffer            buff;
  tf2_ros::TransformListener tfl(buff, nh_);

  ROS_INFO("[kinova_control_manager]: Waiting for arm feedback...");

  while (!getting_joint_angles) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

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

  kinova_msgs::HomeArm msg;
  service_client_homing.call(msg.request, msg.response);
  res.success = true;
  return true;
}
//}

/* callbackPrepareGrippingService //{ */
bool kinova_control_manager::callbackPrepareGrippingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[kinova_control_manager]: Assuming a default gripping pose");
  status              = MotionStatus_t::MOVING;
  time_of_last_motion = ros::Time::now();
  last_goal           = default_gripping_pose;
  goTo(default_gripping_pose);

  while (status != MotionStatus_t::IDLE) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  res.success = true;
  return true;
}
//}

/* callbackAlignArmService //{ */
bool kinova_control_manager::callbackAlignArmService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  status = ALIGNING;
  if (!getting_joint_angles) {
    ROS_ERROR("[kinova_control_manager]: Cannot align arm, internal feedback missing!");
    status      = IDLE;
    res.success = false;
    return false;
  }

  mbzirc_husky_msgs::brickDetect brick_srv;
  brick_srv.request.activate            = true;
  brick_srv.request.groundPlaneDistance = end_effector_pose_raw.pos[2] + arm_base_to_ground;
  service_client_brick_detector.call(brick_srv.request, brick_srv.response);

  ROS_WARN("[kinova_arm_manager]: Waiting for brick detection...");
  ros::Time t_start = ros::Time::now();
  while (!brick_srv.response.activated) {
    ros::Duration(0.1).sleep();
    if ((ros::Time::now().sec - t_start.sec) > 4.0) {
      std::stringstream ss;
      ss << "Failed to launch brick detection!";
      ROS_FATAL("[kinova_arm_manager]: %s", ss.str().c_str());
      status      = IDLE;
      res.success = false;
      res.message = ss.str().c_str();
      return false;
    }
  }


  Eigen::Quaterniond brick_orientation_msg;
  brick_orientation_msg.w()   = brick_srv.response.brickPose.pose.orientation.w;
  brick_orientation_msg.x()   = brick_srv.response.brickPose.pose.orientation.x;
  brick_orientation_msg.y()   = brick_srv.response.brickPose.pose.orientation.y;
  brick_orientation_msg.z()   = brick_srv.response.brickPose.pose.orientation.z;
  Eigen::Vector3d brick_euler = quaternionToEuler(brick_orientation_msg);

  ROS_INFO("[kinova_arm_manager]: Got brick pose: [%.2f, %.2f, %.2f]", brick_srv.response.brickPose.pose.position.x,
           brick_srv.response.brickPose.pose.position.y, brick_euler.z());

  Eigen::Vector3d align(-brick_srv.response.brickPose.pose.position.x, brick_srv.response.brickPose.pose.position.y, brick_euler.z());

  Eigen::Vector3d align_euler = quaternionToEuler(default_gripping_pose.rot);
  Eigen::Vector3d ee_euler    = quaternionToEuler(end_effector_pose_raw.rot);
  align_euler.z()             = ee_euler.z() + brick_euler.z();

  ROS_INFO("[kinova_arm_manager]: Suggested alignment: [%.2f, %.2f, %.2f]", align[0], align[1], align[2]);

  Pose3d new_pose;
  new_pose.pos.x() = end_effector_pose_raw.pos.x() + align.x();
  new_pose.pos.y() = end_effector_pose_raw.pos.y() + align.y();
  new_pose.pos.z() = default_gripping_pose.pos.z();
  new_pose.rot     = eulerToQuaternion(align_euler) * wrist_offset.rot.inverse();
  goTo(new_pose);
  ros::Duration(2.0).sleep();

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

    brick_euler = quaternionToEuler(detected_brick.pose.rot);

    align           = Eigen::Vector3d(-detected_brick.pose.pos.x(), detected_brick.pose.pos.y(), brick_euler.z());
    align_euler     = quaternionToEuler(default_gripping_pose.rot);
    ee_euler        = quaternionToEuler(end_effector_pose_raw.rot);
    align_euler.z() = ee_euler.z() + brick_euler.z();

    ROS_INFO("[kinova_control_manager]: Brick yaw: %.2f", brick_euler.z());
    ROS_INFO("[kinova_arm_manager]: Suggested alignment: [%.2f, %.2f, %.2f]", align[0], align[1], align[2]);

    Pose3d new_pose;
    new_pose.pos.x() = end_effector_pose_raw.pos.x() + align.x();
    new_pose.pos.y() = end_effector_pose_raw.pos.y() + align.y();
    new_pose.pos.z() = default_gripping_pose.pos.z();
    new_pose.rot     = eulerToQuaternion(align_euler) * wrist_offset.rot.inverse();
    goTo(new_pose);
    ros::Duration(1.0).sleep();
    aligned = (align.x() < 0.05 && align.y() < 0.05 && align.z() < 0.05);
  }
  ros::spinOnce();
  ros::Duration(1.0);
  ros::spinOnce();
  status      = IDLE;
  res.success = true;
  return true;
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
  status = PICKING;

  kinova_msgs::PoseVelocity msg;
  ROS_INFO("[kinova_control_manager]: Moving down");

  double stopping_height = (detected_brick.brick_layer * 0.2) - arm_base_to_ground + 0.1;
  std::cout << "Slow down at Z: " << stopping_height << "\n";

  while (!brick_attached && brick_reliable) {

    if (!getting_realsense_brick) {
      ROS_ERROR("[kinova_control_manager]: Brick lost, aborting pickup");
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
      status      = IDLE;
      res.success = false;
      return false;
    }

    msg.twist_linear_x = -detected_brick.pose.pos.x() * linear_vel_modifier;
    msg.twist_linear_y = detected_brick.pose.pos.y() * linear_vel_modifier;
    msg.twist_linear_z = -move_down_speed_faster;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.01).sleep();
  }

  // now the descent will not abort if brick is lost
  ROS_WARN("[kinova_control_manager]: Entering the danger zone!");

  while (!brick_attached && end_effector_pose_compensated.pos.z() > stopping_height) {

    msg.twist_linear_x = 0.0;
    msg.twist_linear_y = 0.0;
    msg.twist_linear_z = -move_down_speed_slower;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.01).sleep();
  }

  grip();
  ROS_WARN("[kinova_control_manager]:MEGA slow now");

  while (!brick_attached) {
    msg.twist_linear_x  = 0.0;
    msg.twist_linear_y  = 0.0;
    msg.twist_linear_z  = -move_down_speed_mega_slow;
    msg.twist_angular_x = 0.0;
    msg.twist_angular_y = 0.0;
    msg.twist_angular_z = 0.0;
    publisher_cartesian_velocity.publish(msg);
    ros::Duration(0.009).sleep();
  }

  msg.twist_linear_x  = 0.0;
  msg.twist_linear_y  = 0.0;
  msg.twist_linear_z  = 0.0;
  msg.twist_angular_x = 0.0;
  msg.twist_angular_y = 0.0;
  msg.twist_angular_z = 0.0;
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
  pose.pos = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  goTo(pose);
  res.success = true;
  return true;
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
  pose.pos = Eigen::Vector3d(req.pose[0], req.pose[1], req.pose[2]);
  pose.rot = eulerToQuaternion(Eigen::Vector3d(req.pose[3], req.pose[4], req.pose[5]));
  goToRelative(pose);
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

  /* idle handler //{ */
  if (status == IDLE) {
    return;
  }
  //}

  if ((ros::Time::now() - time_of_last_motion).sec > no_move_error_timeout) {
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
          if (angular_diff[i] > 3.0) {
            angular_diff[i] = std::abs(angular_diff[i] - 3.0);
          }
        }
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
  brick_attached              = msg->gripping_object;
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
  publishTF();
}
//}

/* goTo //{ */
void kinova_control_manager::goTo(Pose3d pose) {
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
}
//}

/* goToRelative //{ */
/* void kinova_control_manager::goToRelative(Pose3d rel_pose) { */

/*   Pose3d goal_pose = end_effector_pose; */
/*   goal_pose.rot *= wrist_offset.rot; */
/*   goal_pose.pos += goal_pose.rot * wrist_offset.pos; */

/*   goal_pose = goal_pose + rel_pose; */

/*   last_goal = goal_pose; */
/*   last_goal.rot *= wrist_offset.rot.inverse(); */
/*   last_goal.pos -= goal_pose.rot * wrist_offset.pos;  // remove the compensation (this is the actual position of the end effector) */

/*   Eigen::Vector3d euler = quaternionToEuler(rel_pose.rot); */
/*   ROS_INFO("[kinova_control_manager]: Moving end effector to [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", goal_pose.pos.x(), goal_pose.pos.y(), */
/*            goal_pose.pos.z(), euler[0], euler[1], euler[2]); */
/*   ROS_WARN("[kinova_control_manager]: USING VELOCITY CONTROL"); */

/*   Eigen::Vector3d linear_dir     = rel_pose.pos; */
/*   Eigen::Vector3d angular_dir    = quaternionToEuler(rel_pose.rot); */
/*   double          distance       = linear_dir.norm(); */
/*   double          euler_distance = angular_dir.norm(); */

/*   Eigen::Vector3d           move_elem  = (goal_pose.pos - end_effector_pose.pos) * linear_vel_modifier; */
/*   Eigen::Vector3d           euler_elem = quaternionToEuler(goal_pose.rot * end_effector_pose.rot.inverse()); */
/*   kinova_msgs::PoseVelocity msg; */
/*   msg.twist_linear_x  = move_elem.x(); */
/*   msg.twist_linear_y  = move_elem.y(); */
/*   msg.twist_linear_z  = move_elem.z(); */
/*   msg.twist_angular_x = 0.0; */
/*   msg.twist_angular_y = 0.0; */
/*   msg.twist_angular_z = euler_elem.z(); */

/*   double distance_covered       = 0.0; */
/*   double euler_distance_covered = 0.0; */
/*   Pose3d start_pose; */
/*   start_pose.pos = end_effector_pose.pos; */

/*   ros::Time time_start = ros::Time::now(); */
/*   while (distance_covered < distance && euler_distance_covered < euler_distance) { */
/*     publisher_cartesian_velocity.publish(msg); */
/*     ros::Rate(50).sleep(); */
/*     distance_covered       = (end_effector_pose.pos - start_pose.pos).norm(); */
/*     euler_distance_covered = quaternionToEuler(end_effector_pose.rot * start_pose.rot.inverse()).norm(); */
/*     move_elem              = (goal_pose.pos - end_effector_pose.pos) * linear_vel_modifier; */
/*     euler_elem             = quaternionToEuler(goal_pose.rot * end_effector_pose.rot.inverse()); */
/*     if (move_elem.norm() < 0.06) { */
/*       move_elem.normalize(); */
/*       move_elem *= 0.06; */
/*     } */
/*     msg.twist_linear_x  = move_elem.x(); */
/*     msg.twist_linear_y  = move_elem.y(); */
/*     msg.twist_linear_z  = move_elem.z(); */
/*     msg.twist_angular_x = 0.0; */
/*     msg.twist_angular_y = 0.0; */
/*     msg.twist_angular_z = euler_elem.z(); */
/*     ROS_INFO("Moving [%.2f/%.2f], velocity %.2f, angular %.2f", distance_covered, distance, move_elem.norm(), euler_elem.norm()); */

/*     if ((ros::Time::now() - time_start).sec > no_move_error_timeout) { */
/*       ROS_WARN("[kinova_control_manager]: Destination unreachable. Setting zero velocity"); */
/*       msg.twist_linear_x  = 0.0; */
/*       msg.twist_linear_y  = 0.0; */
/*       msg.twist_linear_z  = 0.0; */
/*       msg.twist_angular_x = 0.0; */
/*       msg.twist_angular_y = 0.0; */
/*       msg.twist_angular_z = 0.0; */
/*       publisher_cartesian_velocity.publish(msg); */
/*       return; */
/*     } */
/*   } */
/*   ROS_INFO("[kinova_control_manager]: Goal reached. Setting zero velocity."); */
/*   msg.twist_linear_x  = 0.0; */
/*   msg.twist_linear_y  = 0.0; */
/*   msg.twist_linear_z  = 0.0; */
/*   msg.twist_angular_x = 0.0; */
/*   msg.twist_angular_y = 0.0; */
/*   msg.twist_angular_z = 0.0; */
/*   publisher_cartesian_velocity.publish(msg); */
/* } */

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
  return ((std::abs(b - a) * M_PI) / 180) < nearby_rotation_threshold;
}
//}

/* nearbyAngleRad //{ */
bool kinova_control_manager::nearbyAngleRad(double a, double b) {
  return std::abs(b - a) < nearby_rotation_threshold;
}
//}

/* nearbyPose //{ */
bool kinova_control_manager::nearbyPose(Pose3d p, Pose3d q) {
  double pos_diff = (p.pos - q.pos).norm();
  /* std::cout << "Pos diff: " << pos_diff << "\n"; */

  Eigen::Vector3d p_euler = quaternionToEuler(p.rot);
  Eigen::Vector3d q_euler = quaternionToEuler(q.rot);
  Eigen::Vector3d angular_diff;

  for (int i = 0; i < 3; i++) {
    angular_diff[i] = std::abs(p_euler[i] - q_euler[i]);
  }
  /* std::cout << "Angular diff: " << angular_diff[0] << ", " << angular_diff[1] << ", " << angular_diff[2] << "\n"; */

  return (pos_diff < nearby_position_threshold) && (angular_diff[0] < nearby_rotation_threshold) && (angular_diff[1] < nearby_rotation_threshold) &&
         (angular_diff[2] < nearby_rotation_threshold);
}
//}

/* nearbyVector //{ */
bool kinova_control_manager::nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v) {
  return (u - v).norm() < nearby_position_threshold;
}
//}

/* publishTF //{ */
void kinova_control_manager::publishTF() {
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
  trans.transform.rotation      = tf2::toMsg(end_effector_pose_raw.rot);
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

  if (getting_realsense_brick) {
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
