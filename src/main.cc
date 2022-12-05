#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <iomanip>
#include <iostream>

#include "include/time_sync/time_sync.h"

TimeSync<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud>
    time_sync(5);

void SemanticCallback(const sensor_msgs::Image &msg) {
  std::cout << "semantic img: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;
  time_sync.PushMsg<sensor_msgs::Image>(msg, 0, msg.header.stamp.toSec());
}
void OriginCallback(const sensor_msgs::Image &msg) {
  std::cout << "origin img: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;

  time_sync.PushMsg<sensor_msgs::Image>(msg, 2, msg.header.stamp.toSec());
}
void LinePointsCallback(const sensor_msgs::PointCloud &msg) {
  std::cout << "line points: " << std::setprecision(13)
            << msg.header.stamp.toSec() << std::endl;
  time_sync.PushMsg<sensor_msgs::PointCloud>(msg, 1, msg.header.stamp.toSec());
}

void Process3(const sensor_msgs::Image &img, const sensor_msgs::Image &img1,
              const sensor_msgs::PointCloud &pc) {
  std::cout << "time 2: " << img.header.stamp.toSec() << " / "
            << img1.header.stamp.toSec() << " / " << pc.header.stamp.toSec()
            << std::endl;
}

int main(int argc, char **argv) {
  time_sync.function_call_back_ =
      std::bind(Process3, std::placeholders::_1, std::placeholders::_2,
                std::placeholders::_3);

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