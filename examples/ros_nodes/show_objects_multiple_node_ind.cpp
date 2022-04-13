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
  TCLAP::CmdLine cmd(
      "Subscribe to /velodyne_points topic and show clustering on the data.",
      ' ', "1.0");
  TCLAP::ValueArg<int> angle_arg(
      "", "angle",
      "Threshold angle. Below this value, the objects are separated", false, 10,
      "int");
  TCLAP::ValueArg<int> num_beams_arg(
      "", "num_beams", "Num of vertical beams in laser. One of: [16, 32, 64].",
      true, 0, "int");

  cmd.add(angle_arg);
  cmd.add(num_beams_arg);
  cmd.parse(argc, argv);

  Radians angle_tollerance = Radians::FromDegrees(angle_arg.getValue());

  std::unique_ptr<ProjectionParams> proj_params_ptr = nullptr;
  switch (num_beams_arg.getValue()) {
    case 16:
      proj_params_ptr = ProjectionParams::VLP_16();
      break;
    case 32:
      proj_params_ptr = ProjectionParams::LUMINAR();
      break;
    case 64:
      proj_params_ptr = ProjectionParams::HDL_64();
      break;
  }
  if (!proj_params_ptr) {
    fprintf(stderr,
            "ERROR: wrong number of beams: %d. Should be in [16, 32, 64].\n",
            num_beams_arg.getValue());
    exit(1);
  }

  QApplication application(argc, argv);

  ros::init(argc, argv, "show_objects_node");
  ros::NodeHandle nh;

  string topic_cloud_front = "/luminar_front_points";
  string topic_cloud_left = "/luminar_left_points";
  string topic_cloud_right = "/luminar_right_points";

  CloudOdomRosSubscriber subscriber_front(&nh, *proj_params_ptr, topic_cloud_front);
  CloudOdomRosSubscriber subscriber_left(&nh, *proj_params_ptr, topic_cloud_left);
  CloudOdomRosSubscriber subscriber_right(&nh, *proj_params_ptr, topic_cloud_right);
  Visualizer visualizer;
  //visualizer.show();

  int min_cluster_size = 20;
  int max_cluster_size = 100000;

  int smooth_window_size_front = 7;
  int smooth_window_size_left = 7;
  int smooth_window_size_right = 7;

  Radians ground_remove_angle_front = 18_deg;
  Radians ground_remove_angle_left = 18_deg;
  Radians ground_remove_angle_right = 18_deg;

  auto depth_ground_remover_front = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle_front, smooth_window_size_front);
  auto depth_ground_remover_left = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle_left, smooth_window_size_left);
  auto depth_ground_remover_right = DepthGroundRemover(
      *proj_params_ptr, ground_remove_angle_right, smooth_window_size_right);

  // Create publisher for ground removed cloud, clustered cloud
  CloudRosPublisher cloud_publisher_front(&nh, "luminar_front", "luminar_front_points/ground_removed");
  CloudRosPublisher cloud_publisher_left(&nh, "luminar_left", "luminar_left_points/ground_removed");
  CloudRosPublisher cloud_publisher_right(&nh, "luminar_right", "luminar_right_points/ground_removed");

  // Subscribe clouds
  subscriber_front.AddClient(&depth_ground_remover_front);
  subscriber_left.AddClient(&depth_ground_remover_left);
  subscriber_right.AddClient(&depth_ground_remover_right);

  // Subscribe ground removal
  depth_ground_remover_front.AddClient(&cloud_publisher_front);
  depth_ground_remover_left.AddClient(&cloud_publisher_left);
  depth_ground_remover_right.AddClient(&cloud_publisher_right);

  fprintf(stderr, "INFO: Running with angle tollerance: %f degrees\n",
          angle_tollerance.ToDegrees());

  subscriber_front.StartListeningToRos();
  subscriber_left.StartListeningToRos();
  subscriber_right.StartListeningToRos();
  ros::AsyncSpinner spinner(3);
  spinner.start();

  auto exit_code = application.exec();

  // if we close application, still wait for ros to shutdown
  ros::waitForShutdown();
  return exit_code;
}
