#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ppdr_filter_tools/biquad_filter_cascadeConfig.h>
#include <nav_msgs/Odometry.h>

#include "BiquadFilterCascade.h"

ros::Publisher odom_pub;
BiquadFilterCascade lpf, differentiator;

// void reconfigure_callback(ppdr_filter_tools::biquad_filter_cascadeConfig& config, uint32_t level){
//   return;
// }

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){

  double filtered_pose  = lpf.filter(msg->pose.pose.position.x);
  double filtered_twist = differentiator.filter(msg->pose.pose.position.x);
  
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "robot";
  odom.header.stamp = ros::Time::now();

  odom.pose.pose.position.x = filtered_pose;
  odom.twist.twist.linear.x = filtered_twist;

  odom_pub.publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_filter_node");
  ros::NodeHandle nh("~");

  ros::Subscriber odom_sub = nh.subscribe("odom_with_noise", 10, odom_callback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_filtered", 1);

  // dynamic_reconfigure::Server<ppdr_filter_tools::biquad_filter_cascadeConfig> server;
  // dynamic_reconfigure::Server<ppdr_filter_tools::biquad_filter_cascadeConfig>::CallbackType f;
  // f = boost::bind(&reconfigure_callback, _1, _2);
  // server.setCallback(f);
  
  lpf.add_lpf(50, 5);
  differentiator.add_differentiator(50, 20);
  differentiator.add_lpf(50, 20);
  differentiator.add_lpf(50, 20);

  // differentiator.show_factor();

  ros::spin();
  return 0;
};