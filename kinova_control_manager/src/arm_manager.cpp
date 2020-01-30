#include <mutex>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nodelet/nodelet.h>

#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_arm.h>

#include <std_srvs/Trigger.h>
#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseActionGoal.h>

#include <kinova_control_manager/ArmStatus.h>
#include <kinova_control_manager/EndEffectorPose.h>
#include <kinova_control_manager/Vector3.h>

#include <mrs_msgs/GripperDiagnostics.h>
#include <visualization_msgs/Marker.h>

#define DOF 6
#define ORIGINAL_WRIST_OFFSET Eigen::Vector3d(0.0, 0.0, 0.16)  // [m]

namespace kinova_control_manager
{

typedef enum
{
  MOVING,
  IDLE,
  HOMING,
} MotionStatus_t;

typedef enum
{
  GRIPPER,
  NOZZLE,
} EndEffector_t;

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
  bool            is_initialized           = false;
  bool            getting_joint_angles     = false;
  bool            getting_joint_torques    = false;
  bool            getting_effector_pos     = false;
  bool            getting_gripper_feedback = false;
  bool            brick_attached           = false;
  std::string     arm_type;
  std::mutex      arm_state_mutex;


  // continuous status publishing
  ros::Timer status_timer;
  int        status_timer_rate;
  void       statusTimer(const ros::TimerEvent &evt);

  // goto execution
  void goTo(Pose3d pose);
  void goToRelative(Pose3d pose);

  // configuration params
  Pose3d             home_pose;
  Pose3d             default_gripping_pose;
  Pose3d             default_firefighting_pose;
  Eigen::Quaterniond gripper_down;
  double             nearby_position_threshold;
  double             nearby_rotation_threshold;
  double             no_move_error_timeout;
  EndEffector_t      end_effector_type;

  // arm status
  MotionStatus_t status;
  double         joint_angles[DOF];

  Pose3d    end_effector_pose;
  Pose3d    last_goal;
  double    last_joint_angles[DOF];
  ros::Time time_of_last_motion;

  // geometry & utilities
  bool               nearbyPose(Pose3d p, Pose3d q);
  bool               nearbyAngleDeg(float a, float b);
  bool               nearbyAngleRad(float a, float b);
  bool               nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v);
  Eigen::Vector3d    quaternionToEuler(Eigen::Quaterniond q);
  Eigen::Quaterniond eulerToQuaternion(Eigen::Vector3d euler);
  std::string        statusToString(MotionStatus_t ms);

  // advertised services
  ros::ServiceServer service_server_homing;
  ros::ServiceServer service_server_soft_homing;
  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_relative;
  ros::ServiceServer service_server_prepare_gripping;
  ros::ServiceServer service_server_start_gripping;
  ros::ServiceServer service_server_align_arm;
  ros::ServiceServer service_server_aim_at;

  // internally called services
  ros::ServiceClient service_client_homing;
  ros::ServiceClient service_client_grip;
  ros::ServiceClient service_client_ungrip;

  // subscribers
  ros::Subscriber subscriber_joint_angles;
  ros::Subscriber subscriber_end_effector_pose;
  ros::Subscriber subscriber_gripper_magnet;

  // publishers
  ros::Publisher publisher_arm_status;
  ros::Publisher publisher_end_effector_pose;
  ros::Publisher publisher_dbg_visual;

  // service callbacks
  bool callbackHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackSoftHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGoToService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res);
  bool callbackGoToRelativeService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res);
  bool callbackPrepareGrippingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackStartGripping(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackAlignArmService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackAimAtService(kinova_control_manager::Vector3Request &req, kinova_control_manager::Vector3Response &res);

  // subscriber callbacks
  void callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg);
  void callbackEndEffectorPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg);
  void callbackGripperDiagnosticsTopic(const mrs_msgs::GripperDiagnosticsConstPtr &msg);

  // gripper control
  bool grip();
  bool ungrip();

  // transform
  void                publishTF();
  std::string         husky_base_frame_id;
  std::string         kinova_base_frame_id;
  std::vector<double> husky_to_arm_base_transform;

  tf::TransformBroadcaster tb;
  tf::TransformListener    tl;

  Pose3d husky_to_arm(Pose3d pose_in_husky_frame);
  Pose3d arm_to_husky(Pose3d pose_in_arm_frame);

  Eigen::Quaterniond lookAt(Eigen::Vector3d from, Eigen::Vector3d to);
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
  std::vector<double> firefighting_pose_raw;

  // load params
  nh_.getParam("arm_type", arm_type);
  nh_.getParam("status_timer_rate", status_timer_rate);
  nh_.getParam("no_move_error_timeout", no_move_error_timeout);
  nh_.getParam("nearby_position_threshold", nearby_position_threshold);
  nh_.getParam("nearby_rotation_threshold", nearby_rotation_threshold);
  nh_.getParam("end_effector_home_pose", home_pose_raw);
  nh_.getParam("default_gripping_pose", gripping_pose_raw);
  nh_.getParam("default_firefighting_pose", firefighting_pose_raw);
  nh_.getParam("husky_to_arm_base_transform", husky_to_arm_base_transform);
  nh_.getParam("husky_base_frame_id", husky_base_frame_id);
  nh_.getParam("kinova_base_frame_id", kinova_base_frame_id);

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

  default_firefighting_pose.pos = Eigen::Vector3d(firefighting_pose_raw[0], firefighting_pose_raw[1], firefighting_pose_raw[2]);
  default_firefighting_pose.rot = eulerToQuaternion(Eigen::Vector3d(firefighting_pose_raw[3], firefighting_pose_raw[4], firefighting_pose_raw[5]));

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
  service_server_align_arm        = nh_.advertiseService("align_arm_in", &ArmManager::callbackAlignArmService, this);
  service_server_aim_at           = nh_.advertiseService("aim_at_in", &ArmManager::callbackAimAtService, this);

  // service clients
  service_client_homing = nh_.serviceClient<kinova_msgs::HomeArm>("home_out");
  service_client_grip   = nh_.serviceClient<std_srvs::Trigger>("grip_out");
  service_client_ungrip = nh_.serviceClient<std_srvs::Trigger>("ungrip_out");

  // subscribers
  subscriber_joint_angles      = nh_.subscribe("joint_angles_in", 1, &ArmManager::callbackJointAnglesTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_end_effector_pose = nh_.subscribe("end_effector_pose_in", 1, &ArmManager::callbackEndEffectorPoseTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_gripper_magnet =
      nh_.subscribe("gripper_diagnostics_in", 1, &ArmManager::callbackGripperDiagnosticsTopic, this, ros::TransportHints().tcpNoDelay());

  // publishers
  publisher_arm_status        = nh_.advertise<kinova_control_manager::ArmStatus>("arm_status_out", 1);
  publisher_end_effector_pose = nh_.advertise<kinova_msgs::ArmPoseActionGoal>("end_effector_pose_out", 1);
  publisher_dbg_visual        = nh_.advertise<visualization_msgs::Marker>("target_marker", 1);

  // timers
  status_timer = nh_.createTimer(ros::Rate(status_timer_rate), &ArmManager::statusTimer, this);

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

  if (!getting_joint_angles || !getting_effector_pos) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Reset last arm goal. Homing...");
  status              = MotionStatus_t::HOMING;
  time_of_last_motion = ros::Time::now();
  last_goal           = home_pose;

  kinova_msgs::HomeArm msg;
  service_client_homing.call(msg.request, msg.response);
  res.success = true;
  return true;
}
//}

/* callbackSoftHomingService //{ */
bool ArmManager::callbackSoftHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles || !getting_effector_pos) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Reset last arm goal. Soft homing...");
  status              = MotionStatus_t::HOMING;
  time_of_last_motion = ros::Time::now();
  last_goal           = home_pose;
  goTo(home_pose);

  res.success = true;
  return true;
}
//}

/* callbackPrepareGrippingService //{ */
bool ArmManager::callbackPrepareGrippingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles || !getting_effector_pos) {
    ROS_ERROR("[ArmManager]: Cannot move, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Assuming a default gripping pose");
  status              = MotionStatus_t::MOVING;
  time_of_last_motion = ros::Time::now();
  last_goal           = default_gripping_pose;
  goTo(default_gripping_pose);

  res.success = true;
  return true;
}
//}

/* callbackAlignArmService //{ */
bool ArmManager::callbackAlignArmService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  // TODO
  ROS_WARN("[ArmManager]: Align arm not implemented yet!");
  res.success = false;
  return false;
}
//}

/* callbackAimAtService//{ */
bool ArmManager::callbackAimAtService(kinova_control_manager::Vector3Request &req, kinova_control_manager::Vector3Response &res) {

  if (!getting_joint_angles || !getting_effector_pos) {
    ROS_ERROR("[ArmManager]: Cannot aim at target, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  Eigen::Vector3d target_raw(req.pos[0], req.pos[1], req.pos[2]);
  Pose3d          target_in_husky_frame;
  target_in_husky_frame.pos = target_raw;
  target_in_husky_frame.rot = Eigen::Quaterniond::Identity();

  Pose3d target_in_arm_frame = husky_to_arm(target_in_husky_frame);


  Pose3d firefighting_pose = default_firefighting_pose;
  goTo(firefighting_pose);
  while (nearbyPose(end_effector_pose, firefighting_pose)) {
    ros::Duration(0.2).sleep();
  }

  //TODO write relative DKT
  //use joint 1 for ground plane aim and joint 5 for altitude aim

  /* Eigen::Quaterniond orientation = lookAt(default_firefighting_pose.pos, target_in_arm_frame.pos); */
  /* firefighting_pose.rot          = orientation; */
  /* goTo(firefighting_pose); */

  // adjust for home pose

  Eigen::Vector3d euler = quaternionToEuler(firefighting_pose.rot);
  ROS_INFO_STREAM("[ArmManager]: Assuming position \[" << firefighting_pose.pos[0] << ", " << firefighting_pose.pos[1] << ", " << firefighting_pose.pos[2]
                                                       << ", " << euler[0] << ", " << euler[1] << ", " << euler[2] << "]\n");
  last_goal   = firefighting_pose;
  res.success = true;
  return true;
}
//}

/* callbackStartGripping //{*/
bool ArmManager::callbackStartGripping([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles || !getting_effector_pos) {
    ROS_ERROR("[ArmManager]: Cannot start gripping, internal arm feedback missing!");
    res.success = false;
    return false;
  }

  if (!getting_gripper_feedback) {
    ROS_ERROR("[ArmManager]: Cannot start gripping, gripper feedback missing!");
    res.success = false;
    return false;
  }

  ROS_INFO("[ArmManager]: Turning gripper on");
  grip();

  ROS_INFO("[ArmManager]: Moving down until magnet connects to something");
  while (!brick_attached) {
    if (status != MotionStatus_t::IDLE) {
      std::cout << "Moving\n";
    }
    Pose3d newpose;
    newpose.pos = Eigen::Vector3d(0.0, 0.0, -0.1);
    newpose.rot = Eigen::Quaterniond::Identity();
    goToRelative(newpose);
  }

  ROS_INFO("[ArmManager]: Brick attached. Moving back to the default gripping pose");
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

  if (!getting_joint_angles || !getting_effector_pos) {
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
  status              = MotionStatus_t::MOVING;
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
bool ArmManager::callbackGoToRelativeService(kinova_control_manager::EndEffectorPoseRequest &req, kinova_control_manager::EndEffectorPoseResponse &res) {

  std::scoped_lock lock(arm_state_mutex);
  if (!is_initialized) {
    ROS_ERROR("[ArmManager]: Cannot execute \"goToRelative\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", not initialized!";
    return true;
  }

  if (!getting_joint_angles || !getting_effector_pos) {
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
  status              = MotionStatus_t::MOVING;
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
void ArmManager::callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg) {
  std::scoped_lock lock(arm_state_mutex);
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
    if (!nearbyAngleRad(joint_angles[i], last_joint_angles[i])) {
      /* ROS_INFO("[Arm manager]: Joint %d is moving", i); */
      time_of_last_motion = ros::Time::now();
      return;
    }
  }

  /* idle handler //{ */
  if (status == MotionStatus_t::IDLE) {
    return;
  }
  //}

  if (time_of_last_motion.sec - ros::Time::now().sec > no_move_error_timeout) {

    /* homing handler //{ */
    if (status == MotionStatus_t::HOMING) {
      if (nearbyPose(home_pose, end_effector_pose)) {
        ROS_INFO("[Arm manager]: Homing complete");
      } else {
        ROS_ERROR("[Arm manager]: Homing error! Check arm collisions");
      }
      status = MotionStatus_t::IDLE;
    }
    //}

    /* moving handler //{ */
    if (status == MotionStatus_t::MOVING) {
      if (nearbyPose(last_goal, end_effector_pose)) {
        ROS_INFO("[Arm manager]: Goal reached");
      } else {
        Eigen::Vector3d p_euler = quaternionToEuler(last_goal.rot);
        Eigen::Vector3d q_euler = quaternionToEuler(end_effector_pose.rot);
        Eigen::Vector3d angular_diff;

        for (int i = 0; i < 3; i++) {
          angular_diff[i] = std::abs(p_euler[i] - q_euler[i]);
          if (angular_diff[i] > 3.0) {
            angular_diff[i] = std::abs(angular_diff[i] - 3.0);
          }
        }
        double pos_error = (last_goal.pos - end_effector_pose.pos).norm();
        ROS_WARN("[Arm manager]: Destination unreachable. Position error: %.4f, Rotation error: %.2f, %.2f, %.2f", pos_error, angular_diff[0], angular_diff[1],
                 angular_diff[2]);
      }
      status = MotionStatus_t::IDLE;
    }
    //}
  }
}
//}

/* callbackEndEffectorPoseTopic //{ */
void ArmManager::callbackEndEffectorPoseTopic(const geometry_msgs::PoseStampedConstPtr &msg) {
  std::scoped_lock lock(arm_state_mutex);
  getting_effector_pos = true;

  // !!The message has rotation offset for some reason!!

  end_effector_pose.pos.x() = msg->pose.position.x;
  end_effector_pose.pos.y() = msg->pose.position.y;
  end_effector_pose.pos.z() = msg->pose.position.z;

  end_effector_pose.rot.w() = msg->pose.orientation.w;
  end_effector_pose.rot.x() = msg->pose.orientation.x;
  end_effector_pose.rot.y() = msg->pose.orientation.y;
  end_effector_pose.rot.z() = msg->pose.orientation.z;

  // offset compensation because no wrist is attached
  end_effector_pose.pos -= end_effector_pose.rot * ORIGINAL_WRIST_OFFSET;

  /* tf::StampedTransform trans; */
  /* try { */
  /*   tl.lookupTransform("/j2n6s300_link_base", "/j2n6s300_end_effector", ros::Time(0), trans); */
  /* } */
  /* catch (tf::TransformException ex) { */
  /*   ROS_ERROR("%s", ex.what()); */
  /*   return; */
  /* } */
  /* end_effector_pose.pos.x() = trans.getOrigin().getX(); */
  /* end_effector_pose.pos.y() = trans.getOrigin().getY(); */
  /* end_effector_pose.pos.z() = trans.getOrigin().getZ(); */

  /* Eigen::Quaterniond ee_rot; */
  /* tf::quaternionTFToEigen(trans.getRotation(), ee_rot); */
  /* end_effector_pose.rot = ee_rot; */

  /* end_effector_pose.pos -= end_effector_pose.rot * ORIGINAL_WRIST_OFFSET; */
}
//}

/* callbackGripperDiagnosticsTopic //{ */
void ArmManager::callbackGripperDiagnosticsTopic(const mrs_msgs::GripperDiagnosticsConstPtr &msg) {
  getting_gripper_feedback = true;
  brick_attached           = msg->gripping_object;
}
//}

/* goTo //{ */
void ArmManager::goTo(Pose3d pose) {
  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[ArmManager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(), euler[0],
           euler[1], euler[2]);

  // offset compensation because no wrist is attached
  pose.pos += pose.rot * ORIGINAL_WRIST_OFFSET;

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

  // compensate for original wrist offset
  Pose3d pose = end_effector_pose;
  pose.pos += pose.rot * ORIGINAL_WRIST_OFFSET;

  // add rel_pose to the current end effector pose
  pose = pose + rel_pose;


  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[ArmManager]: Moving end effector by relative [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(), euler[0],
           euler[1], euler[2]);


  last_goal = pose;
  kinova_msgs::ArmPoseActionGoal msg;

  msg.goal.pose.pose.orientation.x = pose.rot.x();
  msg.goal.pose.pose.orientation.y = pose.rot.y();
  msg.goal.pose.pose.orientation.z = pose.rot.z();
  msg.goal.pose.pose.orientation.w = pose.rot.w();


  msg.goal.pose.pose.position.x = pose.pos.x();
  msg.goal.pose.pose.position.y = pose.pos.y();
  msg.goal.pose.pose.position.z = pose.pos.z();


  std::stringstream ss;
  ss << arm_type << "_link_base";
  msg.goal.pose.header.frame_id = ss.str().c_str();
  msg.header.frame_id           = ss.str().c_str();

  publisher_end_effector_pose.publish(msg);
}
//}

/* grip //{ */
bool ArmManager::grip() {
  std_srvs::Trigger trig;
  service_client_grip.call(trig.request, trig.response);
  ROS_INFO_STREAM("[ArmManager]: " << trig.response.message);
  return trig.response.success;
}
//}

/* ungrip //{ */
bool ArmManager::ungrip() {
  std_srvs::Trigger trig;
  service_client_ungrip.call(trig.request, trig.response);
  ROS_INFO_STREAM("[ArmManager]: " << trig.response.message);
  return trig.response.success;
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
  publishTF();
}
//}

/* publishTF //{ */
void ArmManager::publishTF() {
  std::scoped_lock                lock(arm_state_mutex);
  geometry_msgs::TransformStamped trans;
  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = husky_base_frame_id;
  trans.child_frame_id          = kinova_base_frame_id;
  trans.transform.translation.x = husky_to_arm_base_transform[0];
  trans.transform.translation.y = husky_to_arm_base_transform[1];
  trans.transform.translation.z = husky_to_arm_base_transform[2];
  Eigen::Quaterniond rot        = Eigen::Quaterniond::Identity();
  trans.transform.rotation.w    = rot.w();
  trans.transform.rotation.x    = rot.x();
  trans.transform.rotation.y    = rot.y();
  trans.transform.rotation.z    = rot.z();
  tb.sendTransform(trans);

  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = kinova_base_frame_id;
  trans.child_frame_id          = "end_effector_compensated";
  trans.transform.translation.x = end_effector_pose.pos.x();
  trans.transform.translation.y = end_effector_pose.pos.y();
  trans.transform.translation.z = end_effector_pose.pos.z();
  tf::Quaternion tfq;
  tf::quaternionEigenToTF(end_effector_pose.rot, tfq);
  trans.transform.rotation.w = tfq.getW();
  trans.transform.rotation.x = tfq.getX();
  trans.transform.rotation.y = tfq.getY();
  trans.transform.rotation.z = tfq.getZ();
  tb.sendTransform(trans);
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
      /* case MotionStatus_t::GRIPPING: */
      /*   return "GRIPPING"; */
  }
}
//}

/* nearbyVector //{ */
bool ArmManager::nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v) {
  return (u - v).norm() < nearby_position_threshold;
}
//}

/* nearbyPose //{ */
bool ArmManager::nearbyPose(Pose3d p, Pose3d q) {
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

/* nearbyAngleDeg //{ */
bool ArmManager::nearbyAngleDeg(float a, float b) {
  return ((std::abs(b - a) * M_PI) / 180) < nearby_rotation_threshold;
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

/* Coordinate transformations //{ */

/* husky_to_arm //{ */
Pose3d ArmManager::husky_to_arm(Pose3d pose_in_husky_frame) {
  Pose3d transform;
  transform.pos = Eigen::Vector3d(husky_to_arm_base_transform[0], husky_to_arm_base_transform[1], husky_to_arm_base_transform[2]);
  transform.rot = eulerToQuaternion(Eigen::Vector3d(husky_to_arm_base_transform[3], husky_to_arm_base_transform[4], husky_to_arm_base_transform[5]));
  return pose_in_husky_frame - transform;
}
//}

/* arm_to_husky //{ */
Pose3d ArmManager::arm_to_husky(Pose3d pose_in_arm_frame) {
  Pose3d transform;
  transform.pos = Eigen::Vector3d(husky_to_arm_base_transform[0], husky_to_arm_base_transform[1], husky_to_arm_base_transform[2]);
  transform.rot = eulerToQuaternion(Eigen::Vector3d(husky_to_arm_base_transform[3], husky_to_arm_base_transform[4], husky_to_arm_base_transform[5]));
  return pose_in_arm_frame + transform;
}
//}

//}

/* lookAt //{ */
// TODO do not use this, use relative DKT!!!
Eigen::Quaterniond ArmManager::lookAt(Eigen::Vector3d from, Eigen::Vector3d to) {

  if (to[0] < 0) {
    ROS_ERROR("[ArmManager]: Tried pointing at target behind the robot!");
    return Eigen::Quaterniond::Identity();
  }

  double dst = (from - to).norm();

  std::cout << "to: " << to << "\n";
  std::cout << "from: " << from << "\n";
  std::cout << "dst: " << dst << "\n";


  double rot_x = atan2(to[1] - from[1], to[0] - from[0]);
  std::cout << "Rot x: " << rot_x << "\n";
  double rot_z = acos((to[0] - from[0]) / dst);
  std::cout << "Rot z: " << rot_z << "\n";

  Eigen::Vector3d local_x_axis = end_effector_pose.rot * Eigen::Vector3d::UnitX();
  Eigen::Vector3d local_y_axis = end_effector_pose.rot * Eigen::Vector3d::UnitY();
  Eigen::Vector3d local_z_axis = end_effector_pose.rot * Eigen::Vector3d::UnitZ();

  Eigen::Quaterniond q = Eigen::AngleAxisd(rot_x, local_x_axis) * Eigen::AngleAxisd(0, local_y_axis) * Eigen::AngleAxisd(rot_z, local_z_axis);

  Eigen::Vector3d target, offset;
  offset = q * local_y_axis;
  offset = offset * dst;
  target = end_effector_pose.pos + offset;

  visualization_msgs::Marker marker;

  marker.header.frame_id = "j2n6s300_link_base";
  marker.frame_locked    = true;
  marker.header.stamp    = ros::Time::now();
  marker.ns              = "marker_namespace";
  marker.id              = 0;
  marker.action          = visualization_msgs::Marker::ADD;
  marker.type            = visualization_msgs::Marker::LINE_STRIP;
  marker.color.r         = 1.0;
  marker.color.g         = 0.0;
  marker.color.b         = 0.0;
  marker.color.a         = 1.0;
  marker.scale.x         = 0.03;
  marker.scale.y         = 0.03;
  geometry_msgs::Point p1, p2;
  p1.x = from.x();
  p1.y = from.y();
  p1.z = from.z();

  p2.x = target.x();
  p2.y = target.y();
  p2.z = target.z();

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  publisher_dbg_visual.publish(marker);

  return q;
}  // namespace kinova_control_manager
//}

/* vectorAngle //{ */
double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}
//}

}  // namespace kinova_control_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kinova_control_manager::ArmManager, nodelet::Nodelet)
