#include <ros/ros.h>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

void publishTF(tf::TransformBroadcaster tb, Eigen::Vector3d ee_pos, Eigen::Quaterniond ee_rot, Eigen::Vector3d target) {
  geometry_msgs::TransformStamped trans;
  trans.header.stamp            = ros::Time::now();
  trans.header.frame_id         = "base_frame";
  trans.child_frame_id          = "end_effector_frame";
  trans.transform.translation.x = ee_pos[0];
  trans.transform.translation.y = ee_pos[1];
  trans.transform.translation.z = ee_pos[2];
  trans.transform.rotation.w    = ee_rot.w();
  trans.transform.rotation.x    = ee_rot.x();
  trans.transform.rotation.y    = ee_rot.y();
  trans.transform.rotation.z    = ee_rot.z();
  tb.sendTransform(trans);
}

Eigen::Quaterniond lookAt(Eigen::Vector3d from, Eigen::Vector3d to) {

  Eigen::Vector3d forward = (from - to);
  forward.normalize();
  Eigen::Vector3d up    = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d right = forward.cross(up);

  return Eigen::Quaterniond::FromTwoVectors(from, to);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "IntensityFilter");
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  tf::TransformBroadcaster tb;

  Eigen::Vector3d target = Eigen::Vector3d(5.0, 0.0, 0.0);

  Eigen::Vector3d end_effector_pos = Eigen::Vector3d(3.0, 0.0, 1.8);



  while (ros::ok()) {
    publishTF();
    ros::Duration(0.1).sleep();
  }


  ros::spin();

  return 0;
}
