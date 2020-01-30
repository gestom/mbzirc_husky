#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <mbzirc_husky/pickupAction.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <mbzirc_husky/pickupConfig.h>
#include <std_srvs/Trigger.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Arm packages
#include <kinova_driver/kinova_comm.h>
#include <kinova_driver/kinova_arm.h>

#include <kinova_msgs/HomeArm.h>
#include <kinova_msgs/JointAngles.h>
#include <kinova_msgs/PoseVelocity.h>
#include <kinova_msgs/ArmPoseActionGoal.h>

#include <kinova_control_manager/EndEffectorPose.h>

typedef actionlib::SimpleActionServer<mbzirc_husky::pickupAction> Server;
Server*                                                           server;

typedef enum
{
  IDLE,
  ALIGNING_ROBOT,
  ALIGNING_ARM,
  GRASP,
  STACK,
  STOPPING,
  PREEMPTED,
  SUCCESS,
  FAIL
} EPickupState;

EPickupState         state = IDLE;
ros::Publisher       cmd_vel;
ros::Publisher       align_arm;
ros::Subscriber      arm_pose;
ros::Subscriber      arm_force;
ros::Subscriber      robot_pose;
geometry_msgs::Twist base_cmd;

float speedCoef = 1.0;
// float fireThreshold = 45;
/*void callback(mbzirc_husky::pickupConfig &config, uint32_t level) {

        tolerance=config.tolerance;
        angleTolerance=config.angleTolerance;
        distanceTolerance=config.distanceTolerance;
        distance=config.distance;
        fwSpeed=config.fwSpeed;
        minPoints=config.minPoints;
        maxEvalu=config.maxEval;
        spdLimit=config.spdLimit;
        realConst=config.realConst;
        dispConst=config.dispConst;

}*/
/*void poseCallback(const geometry_msgs::Pose::ConstPtr&){

        if (state == ALIGNING_ROBOT){
                //move it closer to box
        }
}*/


bool isTerminal(EPickupState state) {
  if (state == ALIGNING_ROBOT)
    return false;
  if (state == ALIGNING_ARM)
    return false;
  if (state == GRASP)
    return false;
  if (state == STACK)
    return false;
}

void actionServerCallback(const mbzirc_husky::pickupGoalConstPtr& goal, Server* as) {
  mbzirc_husky::pickupResult result;
  state = ALIGNING_ARM;
  while (isTerminal(state) == false) {
    //      if (state == FINAL) state = SUCCESS; else state = FAIL;
    usleep(15000);
  }

  if (state == SUCCESS)
    server->setSucceeded(result);
  if (state == FAIL)
    server->setAborted(result);
  if (state == PREEMPTED)
    server->setPreempted(result);
  state = STOPPING;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pickup");
  ros::NodeHandle n;

  // Dynamic reconfiguration server
  /*dynamic_reconfigure::Server<mbzirc_husky::pickupConfig> dynServer;
         dynamic_reconfigure::Server<mbzirc_husky::pickupConfig>::CallbackType f = boost::bind(&callback, _1, _2);
         dynServer.setCallback(f);*/

  cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  server  = new Server(n, "pickupServer", boost::bind(&actionServerCallback, _1, server), false);
  server->start();
  // scan_sub = n.subscribe("scan", 100, scanCallback);
  // robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
  // point_pub_ = n.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/points2", 100000);
  ros::ServiceClient arm_prepare_client   = n.serviceClient<std_srvs::Trigger>("kinova/arm_manager/prepare_gripping");
  ros::ServiceClient arm_home_hard_client = n.serviceClient<std_srvs::Trigger>("kinova/arm_manager/home_arm");
  ros::ServiceClient arm_home_soft_client = n.serviceClient<std_srvs::Trigger>("kinova/arm_manager/soft_home_arm");

  ros::ServiceClient arm_move_until_obstacle_client = n.serviceClient<std_srvs::Trigger>("kinova/arm_manager/move_down_until_obstacle");
  std_srvs::Trigger  arm_prep_srv;
  std_srvs::Trigger  arm_home_hard_srv;
  std_srvs::Trigger  arm_home_soft_srv;
  std_srvs::Trigger  arm_obstacle_srv;

  while (ros::ok()) {
    if (server->isPreemptRequested() && state != IDLE)
      state = PREEMPTED;
    if (state == STOPPING) {
      base_cmd.linear.x = base_cmd.angular.z = 0;
      cmd_vel.publish(base_cmd);
      state = IDLE;
    }
    if (state == ALIGNING_ARM) {
      if (arm_prepare_client.call(arm_prep_srv)) {
        ROS_INFO("Calling allign arm service");
        state = ALIGNING_ROBOT;
      } else {
        ROS_ERROR("Calling service failed");
        state = FAIL;
      }
    }
    ros::spinOnce();
    usleep(30000);
  }
}

