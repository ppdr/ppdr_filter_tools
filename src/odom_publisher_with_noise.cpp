#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <random>

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle nh("");

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_with_noise", 1);
  ros::Publisher truth_pub = nh.advertise<nav_msgs::Odometry>("/odom_truth", 1);

  ros::Rate rate(50);

  nav_msgs::Odometry odom, odom_truth;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "robot";

  odom_truth = odom;

  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> norm(0, 0.1);

  double omega = 6.28;

  while(ros::ok()){
      ros::spinOnce();

      double t = ros::Time::now().toSec();
      
      // ノイズあり
      odom.pose.pose.position.x = 1.0 * sin(omega*t) + norm(mt);
      // odom.pose.pose.position.x = 1.0 * sin(omega*t) + 0.05*sin(100*t);
      odom.header.stamp = ros::Time::now();
      
      // ノイズなし(真値)
      odom_truth.pose.pose.position.x = 1.0 * sin(omega*t);
      odom_truth.twist.twist.linear.x = omega * cos(omega*t);
      odom_truth.header.stamp = ros::Time::now();

      odom_pub.publish(odom);
      truth_pub.publish(odom_truth);
      rate.sleep();
  }
  return 0;
};