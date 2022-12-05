#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <iomanip>
#include <iostream>

#include "include/time_sync/time_sync.h"

TimeSync<sensor_msgs::Image, sensor_msgs::PointCloud, sensor_msgs::Image>
    time_sync(3, 5);

void SemanticCallback(const sensor_msgs::Image &msg) {
  std::cout << "semantic img: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;
  time_sync.PushMsg0(msg, msg.header.stamp.toSec());
}
void OriginCallback(const sensor_msgs::Image &msg) {
  std::cout << "origin img: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;

  time_sync.PushMsg2(msg, msg.header.stamp.toSec());
}
void LinePointsCallback(const sensor_msgs::PointCloud &msg) {
  std::cout << "line points: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;
  time_sync.PushMsg1(msg, msg.header.stamp.toSec());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pose_graph");
  ros::start();

  ros::NodeHandle nd;

  ros::Subscriber semantic_img_sub_ =
      nd.subscribe("/semantic_image", 1000, SemanticCallback);
  ros::Subscriber origin_img_sub_ =
      nd.subscribe("/original_image", 1000, OriginCallback);
  ros::Subscriber point_cloud_sub_ =
      nd.subscribe("/line_points", 1000, LinePointsCallback);

  // TimeSync<double, double> time_sync(0,1);

  ros::spin();
  ros::shutdown();
  return 0;
}