#include <mutex>

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

#include <mrs_msgs/GripperDiagnostics.h>

#define DOF 6

namespace kinova_control_manager
{

typedef enum
{
  MOVING,
  IDLE,
  HOMING,
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

/* struct Velocity //{ */
struct Velocity
{
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;

  bool operator==(const Velocity &v) const {
    return linear == v.linear && angular == v.angular;
  }

  bool operator!=(const Velocity &v) const {
    return linear != v.linear || angular != v.angular;
  }

  Velocity operator+(const Velocity &v) const {
    Velocity result;
    result.linear + v.linear;
    result.angular + v.angular;
    return result;
  }

  Velocity operator-(const Velocity &v) const {
    Velocity result;
    result.linear - v.linear;
    result.angular - v.angular;
    return result;
  }
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
  bool getting_end_effector        = false;
  bool getting_gripper_diagnostics = false;
  bool getting_realsense_brick     = false;

  std::string arm_type;
  std::string husky_base_frame_id;
  std::string kinova_base_frame_id;

  // continuous status publishing
  ros::Timer status_timer, velocity_republisher;
  int        status_timer_rate;
  void       statusTimer(const ros::TimerEvent &evt);
  void       velocityRepublisher(const ros::TimerEvent &evt);

  // position control
  void goTo(Pose3d pose);
  void goToRelative(Pose3d pose);

  // configuration params
  Pose3d          home_pose;
  Pose3d          default_gripping_pose;
  Pose3d          default_firefighting_pose;
  Eigen::Vector3d original_wrist_offset;

  double nearby_position_threshold;
  double nearby_rotation_threshold;
  double no_move_error_timeout;
  double arm_base_to_ground;
  double linear_vel_modifier;
  double angular_vel_modifier;

  // arm status
  MotionStatus_t status;

  double    joint_angles[DOF];
  double    last_joint_angles[DOF];
  Pose3d    end_effector_pose;
  Pose3d    last_goal;
  Pose3d    brick_pose;
  ros::Time time_of_last_motion;
  Velocity  cartesian_velocity;

  std::mutex brick_pose_mutex;
  std::mutex end_effector_mutex;
  std::mutex joint_angles_mutex;
  std::mutex cartesian_velocity_mutex;

  // advertised services
  ros::ServiceServer service_server_homing;
  ros::ServiceServer service_server_goto;
  ros::ServiceServer service_server_goto_relative;

  // called services
  ros::ServiceClient service_client_homing;

  // publishers
  ros::Publisher publisher_arm_status;
  ros::Publisher publisher_end_effector_pose;
  ros::Publisher publisher_cartesian_velocity;

  // subscribers
  ros::Subscriber subscriber_joint_angles;
  ros::Subscriber subscriber_end_effector;

  // service callbacks
  bool callbackHomingService(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);
  bool callbackGoToRelativeService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res);

  // topic callbacks
  void callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg);
  void callbackEndEffectorTopic(const geometry_msgs::PoseStampedConstPtr &msg);

  // utils
  bool nearbyAngleDeg(float a, float b);
  bool nearbyAngleRad(float a, float b);
  bool nearbyPose(Pose3d p, Pose3d q);
  bool nearbyVector(Eigen::Vector3d u, Eigen::Vector3d v);
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
  nh_.getParam("original_wrist_offset", wrist_offset_raw);

  nh_.getParam("husky_base_frame_id", husky_base_frame_id);
  nh_.getParam("kinova_base_frame_id", kinova_base_frame_id);
  nh_.getParam("arm_base_to_ground", arm_base_to_ground);
  nh_.getParam("linear_vel_modifier", linear_vel_modifier);
  nh_.getParam("angular_vel_modifier", angular_vel_modifier);
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

  if (wrist_offset_raw.size() != 3) {
    ROS_ERROR("[kinova_control_manager]: Parameter \"original_wrist_offset\" expected to have 3 elements, got %ld!", wrist_offset_raw.size());
    ros::shutdown();
  }

  original_wrist_offset = Eigen::Vector3d(wrist_offset_raw[0], wrist_offset_raw[1], wrist_offset_raw[2]);

  //}

  // service servers
  service_server_homing        = nh_.advertiseService("home_in", &kinova_control_manager::callbackHomingService, this);
  service_server_goto          = nh_.advertiseService("goto_in", &kinova_control_manager::callbackGoToService, this);
  service_server_goto_relative = nh_.advertiseService("goto_relative_in", &kinova_control_manager::callbackGoToRelativeService, this);

  // service clients
  service_client_homing = nh_.serviceClient<kinova_msgs::HomeArm>("home_out");

  // publishers
  publisher_end_effector_pose = nh_.advertise<kinova_msgs::ArmPoseActionGoal>("end_effector_pose_out", 1);
  publisher_end_effector_pose = nh_.advertise<kinova_msgs::PoseVelocity>("cartesian_velocity_out", 1);

  // subscribers
  subscriber_joint_angles = nh_.subscribe("joint_angles_in", 1, &kinova_control_manager::callbackJointAnglesTopic, this, ros::TransportHints().tcpNoDelay());
  subscriber_end_effector =
      nh_.subscribe("end_effector_pose_in", 1, &kinova_control_manager::callbackEndEffectorTopic, this, ros::TransportHints().tcpNoDelay());

  // timers
  status_timer         = nh_.createTimer(ros::Rate(status_timer_rate), &kinova_control_manager::statusTimer, this);
  velocity_republisher = nh_.createTimer(ros::Rate(100), &kinova_control_manager::velocityRepublisher, this);

  status = IDLE;

  for (int i = 0; i < DOF; i++) {
    joint_angles[i]      = 0.0;
    last_joint_angles[i] = 0.0;
  }
  ROS_INFO("[kinova_control_manager]: Waiting for arm feedback...");

  while (!getting_joint_angles || !getting_end_effector) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }

  is_initialized = true;
  ROS_INFO("[kinova_control_manager]: Initialized Kinova Arm type: %s", arm_type.c_str());

}  // namespace kinova_control_manager
//}

/* callbackHomingService //{ */
bool kinova_control_manager::callbackHomingService([[maybe_unused]] std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

  if (!getting_joint_angles || !getting_end_effector) {
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

/* callbackGoToService //{ */
bool kinova_control_manager::callbackGoToService(mbzirc_husky_msgs::EndEffectorPoseRequest &req, mbzirc_husky_msgs::EndEffectorPoseResponse &res) {

  std::scoped_lock lock(end_effector_mutex);
  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goTo\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goTo\", not initialized!";
    return false;
  }

  if (!getting_joint_angles || !getting_end_effector) {
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

  std::scoped_lock lock(end_effector_mutex);
  if (!is_initialized) {
    ROS_ERROR("[kinova_control_manager]: Cannot execute \"goToRelative\", not initialized!");
    res.success = false;
    res.message = "Cannot execute \"goToRelative\", not initialized!";
    return false;
  }

  if (!getting_joint_angles || !getting_end_effector) {
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
  goTo(pose);
  res.success = true;
  return true;
}
//}

/* callbackJointAnglesTopic //{ */
void kinova_control_manager::callbackJointAnglesTopic(const kinova_msgs::JointAnglesConstPtr &msg) {
  std::scoped_lock lock(joint_angles_mutex);
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
      time_of_last_motion = ros::Time::now();
      return;
    }
  }

  /* idle handler //{ */
  if (status == IDLE) {
    return;
  }
  //}

  if (time_of_last_motion.sec - ros::Time::now().sec > no_move_error_timeout) {

    /* homing handler //{ */
    if (status == HOMING) {
      if (nearbyPose(home_pose, end_effector_pose)) {
        ROS_INFO("[Arm manager]: Homing complete");
      } else {
        ROS_ERROR("[Arm manager]: Homing error! Check arm collisions");
      }
      status = IDLE;
    }
    //}

    /* moving handler //{ */
    if (status == MOVING) {
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
      status = IDLE;
    }
    //}
  }
}
//}

/* callbackEndEffector //{ */
void kinova_control_manager::callbackEndEffectorTopic(const geometry_msgs::PoseStampedConstPtr &msg) {
  std::scoped_lock lock(end_effector_mutex);
  getting_end_effector = true;

  end_effector_pose.pos.x() = msg->pose.position.x;
  end_effector_pose.pos.y() = msg->pose.position.y;
  end_effector_pose.pos.z() = msg->pose.position.z;

  end_effector_pose.rot.w() = msg->pose.orientation.w;
  end_effector_pose.rot.x() = msg->pose.orientation.x;
  end_effector_pose.rot.y() = msg->pose.orientation.y;
  end_effector_pose.rot.z() = msg->pose.orientation.z;

  // offset compensation because no wrist is attached
  end_effector_pose.pos -= end_effector_pose.rot * original_wrist_offset;
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

/* velocityRepublisher //{ */
void kinova_control_manager::velocityRepublisher([[maybe_unused]] const ros::TimerEvent &evt) {
  if (!is_initialized) {
    return;
  }

  std::scoped_lock lock(cartesian_velocity_mutex);
  if (cartesian_velocity.linear.norm() < 0.001 && cartesian_velocity.angular.norm() < 0.001) {
    return;
  }

  kinova_msgs::PoseVelocity msg;
  msg.twist_linear_x  = cartesian_velocity.linear.x();
  msg.twist_linear_y  = cartesian_velocity.linear.y();
  msg.twist_linear_z  = cartesian_velocity.linear.z();
  msg.twist_angular_x = cartesian_velocity.angular.x();
  msg.twist_angular_y = cartesian_velocity.angular.y();
  msg.twist_angular_z = cartesian_velocity.angular.z();

  publisher_cartesian_velocity.publish(msg);
}
//}

/* goTo //{ */
void kinova_control_manager::goTo(Pose3d pose) {
  Eigen::Vector3d euler = quaternionToEuler(pose.rot);

  ROS_INFO("[kinova_control_manager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", pose.pos.x(), pose.pos.y(), pose.pos.z(),
           euler[0], euler[1], euler[2]);


  last_goal = pose;  // store last_goal before compensation (this is the actual position of the wrist)

  // offset compensation because no wrist is attached
  pose.pos += pose.rot * original_wrist_offset;

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
void kinova_control_manager::goToRelative(Pose3d rel_pose) {

  Pose3d goal_pose = end_effector_pose;
  goal_pose.pos += goal_pose.rot * original_wrist_offset;

  goal_pose = goal_pose + rel_pose;

  last_goal = goal_pose;
  last_goal.pos -= goal_pose.rot * original_wrist_offset;  // remove the compensation (this is the actual position of the end effector)

  Eigen::Vector3d euler = quaternionToEuler(goal_pose.rot);
  ROS_INFO("[kinova_control_manager]: Moving end effector to position [%.3f, %.3f, %.3f], euler [%.3f, %.3f, %.3f]", goal_pose.pos.x(), goal_pose.pos.y(),
           goal_pose.pos.z(), euler[0], euler[1], euler[2]);
  ROS_WARN("[kinova_control_manager]: USING VELOCITY CONTROL");
  Eigen::Vector3d linear_dir  = goal_pose.pos.normalized() * linear_vel_modifier;
  Eigen::Vector3d angular_dir = quaternionToEuler(goal_pose.rot).normalized() * angular_vel_modifier;

  std::scoped_lock lock_ee(end_effector_mutex);
  while (!nearbyPose(end_effector_pose, goal_pose)) {
    linear_dir = (goal_pose.pos - end_effector_pose.pos);
    if (linear_dir.norm() > 1) {
      linear_dir.normalize();
    }
    angular_dir = quaternionToEuler(goal_pose.rot * end_effector_pose.rot.inverse());
    if (angular_dir.norm() > 1) {
      angular_dir.normalize();
    }
    linear_dir *= linear_vel_modifier;
    angular_dir *= angular_vel_modifier;
    ROS_INFO("[kinova_control_manager]: Setting linear velocity [%.2f, %.2f, %.2f], angular [%.2f, %.2f, %.2f]", linear_dir.x(), linear_dir.y(), linear_dir.z(),
             angular_dir.x(), angular_dir.y(), angular_dir.z());
    std::scoped_lock lock_vel(cartesian_velocity_mutex);
    cartesian_velocity.linear  = linear_dir;
    cartesian_velocity.angular = angular_dir;
  }
  ROS_INFO("[kinova_control_manager]: Goal reached! Desired: [%.2f, %.2f, %.2f], Actual: [%.2f, %.2f, %.2f]", goal_pose.pos.x(), goal_pose.pos.y(),
           goal_pose.pos.z(), end_effector_pose.pos.x(), end_effector_pose.pos.y(), end_effector_pose.pos.z());

  std::scoped_lock lock_vel(cartesian_velocity_mutex);
  cartesian_velocity.linear  = Eigen::Vector3d::Zero();
  cartesian_velocity.angular = Eigen::Vector3d::Zero();
  ROS_INFO("[kinova_control_manager]: Set zero velocity");
}
//}

/* nearbyAngleDeg //{ */
bool kinova_control_manager::nearbyAngleDeg(float a, float b) {
  return ((std::abs(b - a) * M_PI) / 180) < nearby_rotation_threshold;
}
//}

/* nearbyAngleRad //{ */
bool kinova_control_manager::nearbyAngleRad(float a, float b) {
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

}  // namespace kinova_control_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(kinova_control_manager::kinova_control_manager, nodelet::Nodelet)
