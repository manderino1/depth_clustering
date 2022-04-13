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

#include "ros_bridge/cloud_sync_ros_subscriber.h"
#include <eigen_conversions/eigen_msg.h>

#include <vector>
#include <string>
#include <algorithm>

#include "utils/pose.h"

namespace depth_clustering {

using ros::NodeHandle;
using message_filters::Subscriber;
using message_filters::Synchronizer;
using message_filters::sync_policies::ApproximateTime;
using nav_msgs::Odometry;
using sensor_msgs::PointCloud2;
using sensor_msgs::PointCloud2ConstPtr;

using std::vector;
using std::string;
using std::map;

template <class T>
T BytesTo(const vector<uint8_t>& data, uint32_t start_idx) {
  const size_t kNumberOfBytes = sizeof(T);
  uint8_t byte_array[kNumberOfBytes];
  // forward bit order (it is a HACK. We do not account for bigendianes)
  for (size_t i = 0; i < kNumberOfBytes; ++i) {
    byte_array[i] = data[start_idx + i];
  }
  T result;
  std::copy(reinterpret_cast<const uint8_t*>(&byte_array[0]),
            reinterpret_cast<const uint8_t*>(&byte_array[kNumberOfBytes]),
            reinterpret_cast<uint8_t*>(&result));
  return result;
}

CloudSyncRosSubscriber::CloudSyncRosSubscriber(ros::NodeHandle* node_handle,
                                              const ProjectionParams& params,
                                              const std::string& topic_cloud_1,
                                              const std::string& topic_cloud_2,
                                              const std::string& topic_cloud_3)
    : AbstractSender{SenderType::STREAMER}, _params{params} {
  _node_handle = node_handle;
  _topic_cloud_1 = topic_cloud_1;
  _topic_cloud_2 = topic_cloud_2;
  _topic_cloud_3 = topic_cloud_3;
  _msg_queue_size = 10;

  _subscriber_cloud_1 = nullptr;
  _subscriber_cloud_2 = nullptr;
  _subscriber_cloud_3 = nullptr;
  _sync = nullptr;
}

void CloudSyncRosSubscriber::StartListeningToRos() {
  _subscriber_cloud_1 = new Subscriber<PointCloud2>(
      *_node_handle, _topic_cloud_1, _msg_queue_size);
  _subscriber_cloud_2 = new Subscriber<PointCloud2>(
      *_node_handle, _topic_cloud_2, _msg_queue_size);
  _subscriber_cloud_3 = new Subscriber<PointCloud2>(
      *_node_handle, _topic_cloud_3, _msg_queue_size);
  _sync = new Synchronizer<ApproximateTimePolicy>(
      ApproximateTimePolicy(10), *_subscriber_cloud_1, *_subscriber_cloud_2, *_subscriber_cloud_3);
  _sync->registerCallback(
      boost::bind(&CloudSyncRosSubscriber::Callback, this, _1, _2, _3));
}

void CloudSyncRosSubscriber::Callback(const PointCloudT::ConstPtr& msg_cloud_1,
                                      const PointCloudT::ConstPtr& msg_cloud_2,
                                      const PointCloudT::ConstPtr& msg_cloud_3) {
  // Convert cloud
  Cloud::Ptr cloud_1_ptr = RosCloudToCloud(msg_cloud_1);
  Cloud::Ptr cloud_2_ptr = RosCloudToCloud(msg_cloud_2);
  Cloud::Ptr cloud_3_ptr = RosCloudToCloud(msg_cloud_3);

  // Project cloud
  cloud_1_ptr->InitProjection(_params);
  cloud_2_ptr->InitProjection(_params);
  cloud_3_ptr->InitProjection(_params);

  // Set frame ids
  cloud_1_ptr->SetFrameId(msg_cloud_1->header.frame_id);
  cloud_2_ptr->SetFrameId(msg_cloud_2->header.frame_id);
  cloud_3_ptr->SetFrameId(msg_cloud_3->header.frame_id);

  // Share cloud
  ShareDataWithAllClients(*cloud_1_ptr);
  ShareDataWithAllClients(*cloud_2_ptr);
  ShareDataWithAllClients(*cloud_3_ptr);
}

Cloud::Ptr CloudSyncRosSubscriber::RosCloudToCloud(
    const PointCloud2::ConstPtr& msg) {
  uint32_t x_offset = msg->fields[0].offset;
  uint32_t y_offset = msg->fields[1].offset;
  uint32_t z_offset = msg->fields[2].offset;
  uint32_t ring_offset = msg->fields[4].offset;

  // Initialize all ring numbers to not found
  bool ring_present[128] = {false};

  Cloud cloud;
  for (uint32_t point_start_byte = 0, counter = 0;
       point_start_byte < msg->data.size();
       point_start_byte += msg->point_step, ++counter) {
    RichPoint point;
    point.x() = BytesTo<float>(msg->data, point_start_byte + x_offset);
    point.y() = BytesTo<float>(msg->data, point_start_byte + y_offset);
    point.z() = BytesTo<float>(msg->data, point_start_byte + z_offset);
    //point.ring() = BytesTo<uint16_t>(msg->data, point_start_byte + ring_offset);
    point.ring() = BytesTo<float>(msg->data, point_start_byte + ring_offset);

    ring_present[point.ring()] = true;

    // point.z *= -1;  // hack
    cloud.push_back(point);
  }

  // Find if both last & first are present, if yes there is a jump
  int min_ring = 0;
  int max_ring = 127;
  if(ring_present[0] && ring_present[127]) {
    // Jump is present, find first and last
    // Find max ring
    for(int i=0; i<128; i++) {
      if(!ring_present[i]) {
        max_ring = i-1;
        break;
      }
    }
    // Find min ring
    for(int i=127; i>=0; i--) {
      if(!ring_present[i]) {
        min_ring = i+1;
        break;
      }
    }
  } else {
    // No jump, find first and last
    bool min_found = false;
    for(int i=0; i<128; i++) {
      if(!min_found) {
        if(ring_present[i]) {
          min_ring = i;
          min_found = true;
        }
      } else {
        if(!ring_present[i]) {
          max_ring = i-1;
          break;
        }
      }
    }
  }

  // TODO(marco): this is inefficient, do this in a single step with the previous loop
  Cloud cloud_realigned;
  for(auto &point : cloud.points()) {
    RichPoint new_point = point;

    if(min_ring < max_ring) {
      if(point.ring() < min_ring || point.ring() > max_ring) {
        continue;
      }
    } else {
      if(point.ring() < min_ring && point.ring() > max_ring) {
        continue;
      }
    }

    // Calculate the new ring
    if(min_ring < max_ring) { // Rings are in order between 0 and 128
      new_point.ring() = 33-(point.ring() - min_ring)/2;
    } else { // Points get to 127 and back from 0
      if(point.ring() >= min_ring) { // Between min and 127
        new_point.ring() = 33-(point.ring() - min_ring)/2;
      } else { // Between 0 and max
        new_point.ring() = 33-(point.ring() + (128-min_ring))/2;
      }
    }

    // TODO(marco): this condition should never be met but happens because number of layers are not constant
    if(new_point.ring() >=34) {
      std::cout << new_point.ring() << std::endl;
      continue;
    }

    cloud_realigned.push_back(new_point);
  }

  return make_shared<Cloud>(cloud_realigned);
}

}  // namespace depth_clustering
