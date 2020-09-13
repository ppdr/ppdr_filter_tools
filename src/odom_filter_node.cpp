#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ppdr_filter_tools/filter_paramConfig.h>
#include <nav_msgs/Odometry.h>

#include "BiquadFilter.h"

ros::Publisher odom_pub;
BiquadFilter lpf, differentiator;
double forget_factor = 0.8;

void reconfigure_callback(ppdr_filter_tools::filter_paramConfig& config, uint32_t level){

  forget_factor = config.forget_factor;
  lpf.init_lpf(config.lpf_sampling_freq, config.lpf_cutoff_freq);
  differentiator.init_differentiator(config.lpf_sampling_freq, config.lpf_cutoff_freq);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){
  static double filtered_forget = 0;
  filtered_forget = forget_factor*(filtered_forget) + (1-forget_factor)*msg->pose.pose.position.x;

  double filtered = lpf.filter(msg->pose.pose.position.x);
  double filtered_twist = differentiator.filter(msg->pose.pose.position.x);
  
  nav_msgs::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "robot";
  odom.header.stamp = ros::Time::now();

  odom.pose.pose.position.x = filtered;
  odom.pose.pose.position.y = filtered_forget;
  odom.twist.twist.linear.x = filtered_twist;

  odom_pub.publish(odom);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_filter_node");
  ros::NodeHandle nh("~");

  ros::Subscriber odom_sub = nh.subscribe("odom_with_noise", 10, odom_callback);
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom_filtered", 1);

  dynamic_reconfigure::Server<ppdr_filter_tools::filter_paramConfig> server;
  dynamic_reconfigure::Server<ppdr_filter_tools::filter_paramConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1, _2);
  server.setCallback(f);

  lpf.init_lpf(50, 4);
  differentiator.init_differentiator(50, 4);

  ros::spin();
  return 0;
};