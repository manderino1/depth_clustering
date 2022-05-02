// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

#include <ros/ros.h>

#include <qapplication.h>

#include <string>

#include "ros_bridge/cloud_odom_ros_subscriber.h"

#include "clusterers/image_based_clusterer.h"
#include "ground_removal/depth_ground_remover.h"
#include "projections/ring_projection.h"
#include "projections/spherical_projection.h"
#include "utils/radians.h"
#include "visualization/visualizer.h"

#include "tclap/CmdLine.h"
#include "ros_bridge/cloud_ros_publisher.h"
#include "ros_bridge/cloud_clusters_ros_publisher.h"

using std::string;

using namespace depth_clustering;

using ClustererT = ImageBasedClusterer<LinearImageLabeler<>>;

int main(int argc, char* argv[]) {
  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;
  proj_params_ptr = ProjectionParams::LUMINAR();

  ros::init(argc, argv, "show_objects_node");
  ros::NodeHandle nh("~");

  string topic_cloud_front = "/luminar_front_points";
  string topic_cloud_left = "/luminar_left_points";
  string topic_cloud_right = "/luminar_right_points";

  CloudOdomRosSubscriber subscriber_front(&nh, *proj_params_ptr, topic_cloud_front);
  CloudOdomRosSubscriber subscriber_left(&nh, *proj_params_ptr, topic_cloud_left);
  CloudOdomRosSubscriber subscriber_right(&nh, *proj_params_ptr, topic_cloud_right);

  int smooth_window_size_front;
  int smooth_window_size_left;
  int smooth_window_size_right;

  double ground_remove_angle_front;
  double ground_remove_angle_left;
  double ground_remove_angle_right;

  double angle_tollerance_front;
  double angle_tollerance_left;
  double angle_tollerance_right;

  int min_cluster_size_front;
  int min_cluster_size_left;
  int min_cluster_size_right;

  int max_cluster_size_front;
  int max_cluster_size_left;
  int max_cluster_size_right;

  std::string topic_prefix;

  nh.param("smooth_window_size_front", smooth_window_size_front, 5);
  nh.param("smooth_window_size_left", smooth_window_size_left, 5);
  nh.param("smooth_window_size_right", smooth_window_size_right, 5);

  nh.param("ground_remove_angle_front", ground_remove_angle_front, 20.0);
  nh.param("ground_remove_angle_left", ground_remove_angle_left, 20.0);
  nh.param("ground_remove_angle_right", ground_remove_angle_right, 20.0);

  nh.param("angle_tollerance_front", angle_tollerance_front, 10.0);
  nh.param("angle_tollerance_left", angle_tollerance_left, 10.0);
  nh.param("angle_tollerance_right", angle_tollerance_right, 10.0);

  nh.param("min_cluster_size_front", min_cluster_size_front, 10);
  nh.param("min_cluster_size_left", min_cluster_size_left, 10);
  nh.param("min_cluster_size_right", min_cluster_size_right, 10);

  nh.param("max_cluster_size_front", max_cluster_size_front, 20000);
  nh.param("max_cluster_size_left", max_cluster_size_left, 20000);
  nh.param("max_cluster_size_right", max_cluster_size_right, 20000);

  nh.param("topic_prefix", topic_prefix, std::string("5_20"));

  ROS_INFO("Smooth Window Size Front %i", smooth_window_size_front);
  ROS_INFO("Smooth Window Size Left %i", smooth_window_size_left);
  ROS_INFO("Smooth Window Size Right %i", smooth_window_size_right);
  ROS_INFO("Ground Remove Angle Front %f", ground_remove_angle_front);
  ROS_INFO("Ground Remove Angle Left %f", ground_remove_angle_left);
  ROS_INFO("Ground Remove Angle Right %f", ground_remove_angle_right);
  ROS_INFO("Topic Prefix %s", topic_prefix.c_str());


  auto depth_ground_remover_front = DepthGroundRemover(
      *proj_params_ptr, Radians::FromDegrees(ground_remove_angle_front), smooth_window_size_front);
  auto depth_ground_remover_left = DepthGroundRemover(
      *proj_params_ptr, Radians::FromDegrees(ground_remove_angle_left), smooth_window_size_left);
  auto depth_ground_remover_right = DepthGroundRemover(
      *proj_params_ptr, Radians::FromDegrees(ground_remove_angle_right), smooth_window_size_right);

  ClustererT clusterer_front(Radians::FromDegrees(angle_tollerance_front), min_cluster_size_front, max_cluster_size_front);
  ClustererT clusterer_left(Radians::FromDegrees(angle_tollerance_left), min_cluster_size_left, max_cluster_size_left);
  ClustererT clusterer_right(Radians::FromDegrees(angle_tollerance_right), min_cluster_size_right, max_cluster_size_right);
  // TODO: angles is def option but could fail with custom range image, check its behaviour
  clusterer_front.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer_left.SetDiffType(DiffFactory::DiffType::ANGLES);
  clusterer_right.SetDiffType(DiffFactory::DiffType::ANGLES);


  // Create publisher for ground removed cloud, clustered cloud
  CloudRosPublisher cloud_publisher_front(&nh, "luminar_front", "/luminar_front_points/ground_removed/c" + topic_prefix);
  CloudRosPublisher cloud_publisher_left(&nh, "luminar_left", "/luminar_left_points/ground_removed/c" + topic_prefix);
  CloudRosPublisher cloud_publisher_right(&nh, "luminar_right", "/luminar_right_points/ground_removed/c" + topic_prefix);

  // Create publisher for clustering
  CloudClusterRosPublisher cluster_publisher_front(&nh, "luminar_front", "/luminar_front_points/clusters/c" + topic_prefix);
  CloudClusterRosPublisher cluster_publisher_left(&nh, "luminar_left", "/luminar_left_points/clusters/c" + topic_prefix);
  CloudClusterRosPublisher cluster_publisher_right(&nh, "luminar_right", "/luminar_right_points/clusters/c" + topic_prefix);

  // Subscribe clouds
  subscriber_front.AddClient(&depth_ground_remover_front);
  subscriber_left.AddClient(&depth_ground_remover_left);
  subscriber_right.AddClient(&depth_ground_remover_right);

  // Subscribe ground removal
  depth_ground_remover_front.AddClient(&cloud_publisher_front);
  depth_ground_remover_left.AddClient(&cloud_publisher_left);
  depth_ground_remover_right.AddClient(&cloud_publisher_right);

  depth_ground_remover_front.AddClient(&clusterer_front);
  depth_ground_remover_left.AddClient(&clusterer_left);
  depth_ground_remover_right.AddClient(&clusterer_right);

  // Subscribe clustering
  clusterer_front.AddClient(&cluster_publisher_front);
  clusterer_left.AddClient(&cluster_publisher_left);
  clusterer_right.AddClient(&cluster_publisher_right);

  subscriber_front.StartListeningToRos();
  subscriber_left.StartListeningToRos();
  subscriber_right.StartListeningToRos();
  ros::AsyncSpinner spinner(3);
  spinner.start();

  // if we close application, still wait for ros to shutdown
  ros::waitForShutdown();
  return 0;
}
