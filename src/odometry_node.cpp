#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Odometry.h>

odometry_namespace::Odometry odometry;
std::string base_frame_id;
std::string odom_frame_id;
ros::Publisher odom_pub;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  tf2::Quaternion q;
  q.setValue(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // 提取欧拉角
  Pose pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.theta = yaw;
  odometry.setPose(pose);
}

void rwCallback(const std_msgs::Int32& msg) {
  odometry.updateRightWheel(msg.data);
}

void lwCallback(const std_msgs::Int32& msg) {
  odometry.updateLeftWheel(msg.data);
}

void odom_publish(tf2_ros::TransformBroadcaster& broadcaster) {
  odometry.updatePose(ros::Time::now());
  const Pose pose = odometry.getPose();
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  q.normalize();
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = ros::Time::now();
  transform.transform.translation.x = pose.x;
  transform.transform.translation.y = pose.y;
  transform.transform.translation.z = 0;
  transform.transform.rotation.x = q[0];
  transform.transform.rotation.y = q[1];
  transform.transform.rotation.z = q[2];
  transform.transform.rotation.w = q[3];
  transform.header.frame_id = odom_frame_id;
  transform.child_frame_id = base_frame_id;
  const double pose_length = sqrt(pose.x * pose.x + pose.y * pose.y);
  if (pose_length > 100)
    ROS_WARN("Length too long, now %lf", pose_length);
  broadcaster.sendTransform(transform);

  nav_msgs::Odometry odom_data;
  odom_data.header.stamp = ros::Time::now();
  odom_data.header.frame_id = odom_frame_id;
  odom_data.child_frame_id = base_frame_id;
  odom_data.pose.pose.position.x = pose.x;
  odom_data.pose.pose.position.y = pose.y;
  odom_data.pose.pose.orientation.x = q[0];
  odom_data.pose.pose.orientation.y = q[1];
  odom_data.pose.pose.orientation.z = q[2];
  odom_data.pose.pose.orientation.w = q[3];
  odom_data.twist.twist.linear.x = pose.xVel;
  odom_data.twist.twist.angular.z = pose.thetaVel;
  odom_pub.publish(odom_data);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "odometry_node");
  tf2_ros::TransformBroadcaster broadcaster;
  int32_t ticks_per_meter;
#ifndef TICKS_PER_METER
  if(!ros::param::get("~ticks_per_meter", ticks_per_meter)) {
    ROS_ERROR("Cannot get ticks_per_meter param");
    return 1;
  }
#else
  ticks_per_meter = TICKS_PER_METER
#endif

  double wheel_separation;
#ifndef WHEEL_SEPARATION
  if(!ros::param::get("~wheel_separation", wheel_separation)) {
    ROS_ERROR("Cannot get wheel_separation param");
    return 2;
  }
#else
  wheel_separation = WHEEL_SEPARATION;
#endif

  double rate;
  if(!ros::param::get("~rate", rate))
    rate = 10;

  if(!ros::param::get("~base_frame_id", base_frame_id))
    base_frame_id = "base_link";

  if(!ros::param::get("~odom_frame_id", odom_frame_id))
    odom_frame_id = "odom";

  int32_t encoder_min, encoder_max;
  if(!ros::param::get("~encoder_min", encoder_min))
    encoder_min = -32768;
  if(!ros::param::get("~encoder_max", encoder_max))
    encoder_min = 32767;


  ros::NodeHandle node_handle;
  odom_pub = node_handle.advertise<nav_msgs::Odometry>("/odom", 2);
  ros::Subscriber ip_sub = node_handle.subscribe("/initialpose", 2, initialPoseCallback);
  ros::Subscriber rw_sub = node_handle.subscribe("/rwheel_ticks", 2, rwCallback);
  ros::Subscriber lw_sub = node_handle.subscribe("/lwheel_ticks", 2, lwCallback);

  odometry.setTicksPerMeter(ticks_per_meter);
  odometry.setWheelSeparation(wheel_separation);
  odometry.setEncoderRange(encoder_min, encoder_max);
  odometry.setTime(ros::Time::now());
  ros::Rate srate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    odom_publish(broadcaster);
    srate.sleep();
  }
  return 0;
}

