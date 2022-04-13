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

#include "projections/luminar_projection.h"

#include <vector>

#include "utils/timer.h"

namespace depth_clustering {

cv::Mat& LuminarProjection::depth_image_indexes() { return this->depth_image_indexes_; }
cv::Mat& LuminarProjection::depth_image_pitch() { return this->depth_image_pitch_; }

void LuminarProjection::InitFromPoints(const RichPoint::AlignedVector& points) {
  fprintf(stderr, "Projecting cloud with %lu points\n", points.size());
  time_utils::Timer timer;
  this->CheckCloudAndStorage(points);
  // share ownership of input cloud
  for (size_t index = 0; index < points.size(); ++index) {
    const auto& point = points[index];
    float dist_to_sensor = point.DistToSensor2D();
    if (dist_to_sensor < 0.01f) {
      continue;
    }
    auto angle_cols = Radians::FromRadians(atan2(point.y(), point.x()));
    size_t bin_cols = _params.ColFromAngle(angle_cols);
    size_t bin_rows = point.ring();
    // adding point pointer
    this->_data[bin_cols][bin_rows].points().push_back(index);
    auto& current_written_depth =
        this->_depth_image.at<float>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
      // write index
      this->depth_image_indexes_.at<int>(bin_rows, bin_cols) = static_cast<int>(index);
    }
  }

  calculatePitch(points);
  fprintf(stderr, "Cloud projected in %lu us\n", timer.measure());
}

CloudProjection::Ptr LuminarProjection::Clone() const {
  return CloudProjection::Ptr(new LuminarProjection(*this));
}

RichPoint LuminarProjection::UnprojectPoint(const cv::Mat& image, const int row,
                                         const int col) const {
  RichPoint point = CloudProjection::UnprojectPoint(image, row, col);
  point.ring() = row;
  return point;
}

void LuminarProjection::calculatePitch(const RichPoint::AlignedVector& points) {
  for (int r = 0; r < _depth_image.rows; ++r) {
    for (int c = 0; c < _depth_image.cols; ++c) {
      int index = this->depth_image_indexes().at<int>(r, c);
      if(index != -1) {
        // If cell is full fill with its pitch
        RichPoint point = points.at(index);
        float pitch = M_PI/2 - atan2(sqrt(pow(point.x(),2) + pow(point.y(),2)), point.z());
        this->depth_image_pitch_.at<float>(r,c) = pitch;
      } else {
        // If cell is empty interpolate pitch

        // Moving left to right, so if cell to the left has no pitch this will have no pitch
        if(c == 0 || isnan(this->depth_image_pitch_.at<float>(r,c-1))) {
          continue;
        }

        float pitch = interpolatePitch(r, c, _depth_image.cols, points);
        this->depth_image_pitch_.at<float>(r,c) = pitch;
      }
    }
  }
}

float LuminarProjection::interpolatePitch(int row, int col, int image_cols, const RichPoint::AlignedVector& points) {
  // Search first filled point on the left
  float pitch_left = NAN;
  int col_left = -1;
  for(int i=col; i>=0; i--) {
    int index = this->depth_image_indexes().at<int>(row, i);
    if(index != -1) {
      RichPoint point_left = points.at(index);
      col_left = i;
      pitch_left = M_PI/2 - atan2(sqrt(pow(point_left.x(),2) + pow(point_left.y(),2)), point_left.z());
      break;
    }
  }

  // Search first filled point on the right
  float pitch_right = NAN;
  int col_right = -1;
  for(int i=col; i<image_cols; i++) {
    int index = this->depth_image_indexes().at<int>(row, i);
    if(index != -1) {
      RichPoint point_left = points.at(index);
      col_right = i;
      pitch_right = M_PI/2 - atan2(sqrt(pow(point_left.x(),2) + pow(point_left.y(),2)), point_left.z());
      break;
    }
  }

  // If both were found interpolate
  if(col_left != -1 && col_right != -1) {
    float percentage_left = float(abs(col - col_left))/float((abs(col - col_left) + abs(col - col_right)));
    float percentage_right = 1-percentage_left;
    return(pitch_left*percentage_left + pitch_right*percentage_right);
  } else {
    return(NAN);
  }
}

}  // namespace depth_clustering
