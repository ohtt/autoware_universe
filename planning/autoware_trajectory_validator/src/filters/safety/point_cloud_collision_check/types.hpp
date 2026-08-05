// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_

#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;
using Point2d = autoware_utils_geometry::Point2d;
using Polygon2d = autoware_utils_geometry::Polygon2d;

// motion_velocity_obstacle_stop_module/types.hpp:37-112
/// @brief Stop obstacle label. Only POINTCLOUD is produced here, but the full set is kept because
/// the assumed deceleration in calc_braking_dist_along_trajectory is selected per label.
struct StopObstacleClassification
{
  enum class Type {
    UNKNOWN,
    CAR,
    TRUCK,
    BUS,
    TRAILER,
    MOTORCYCLE,
    BICYCLE,
    PEDESTRIAN,
    ANIMAL,
    HAZARD,
    POINTCLOUD
  };

  StopObstacleClassification() = default;
  explicit StopObstacleClassification(Type v) : label(v) {}

  Type label{};
};

// motion_velocity_obstacle_stop_module/types.hpp:116-120
struct CollisionPointWithDist
{
  geometry_msgs::msg::Point point{};
  double dist_to_collide{};
};

// motion_velocity_obstacle_stop_module/types.hpp:122-128
/// @brief One tracked point cloud obstacle. The velocity is not observable from a single frame, so
/// it is estimated by associating collision points across cycles and low-pass filtering them.
struct PointcloudStopCandidate
{
  std::vector<double> initial_velocities{};
  autoware::signal_processing::LowpassFilter1d vel_lpf{0.0};
  rclcpp::Time latest_collision_pointcloud_time;
  CollisionPointWithDist latest_collision_point;
};

// motion_velocity_obstacle_stop_module/types.hpp:130-141
struct PolygonParam
{
  std::optional<double> trimming_length{};
  double lateral_margin{};
  double off_track_scale{};

  // Ordering exists only to use PolygonParam as a std::map key for the polygon cache.
  bool operator<(const PolygonParam & other) const
  {
    return std::tie(trimming_length, lateral_margin, off_track_scale) <
           std::tie(other.trimming_length, other.lateral_margin, other.off_track_scale);
  }
};

// motion_velocity_obstacle_stop_module/types.hpp:164-183 (the pointcloud constructor)
struct StopObstacle
{
  StopObstacle(
    const rclcpp::Time & arg_stamp, const StopObstacleClassification & arg_object_classification,
    const double arg_lon_velocity, const geometry_msgs::msg::Point & arg_collision_point,
    const double arg_dist_to_collide_on_decimated_traj, const PolygonParam & arg_polygon_param,
    const std::optional<double> arg_braking_dist = std::nullopt)
  : stamp(arg_stamp),
    velocity(arg_lon_velocity),
    collision_point(arg_collision_point),
    dist_to_collide_on_decimated_traj(arg_dist_to_collide_on_decimated_traj),
    classification(arg_object_classification),
    polygon_param(arg_polygon_param),
    braking_dist(arg_braking_dist)
  {
    if (arg_object_classification.label != StopObstacleClassification::Type::POINTCLOUD) {
      throw std::invalid_argument("StopObstacle must be constructed with the POINTCLOUD label");
    }
  }

  rclcpp::Time stamp;
  double velocity;
  geometry_msgs::msg::Point collision_point;
  double dist_to_collide_on_decimated_traj;
  StopObstacleClassification classification;
  PolygonParam polygon_param;
  std::optional<double> braking_dist;
};

// motion_velocity_obstacle_stop_module/types.hpp:200-211
/// @brief Decimated trajectory points paired with the one-step ego footprint polygon of each point.
struct DetectionPolygon
{
  const std::vector<TrajectoryPoint> traj_points;
  const std::vector<Polygon2d> polygons;

  DetectionPolygon(std::vector<TrajectoryPoint> && points, std::vector<Polygon2d> && polys)
  : traj_points(std::move(points)), polygons(std::move(polys))
  {
    if (traj_points.size() != polygons.size()) {
      throw std::invalid_argument("Vector sizes must be identical for DetectionPolygon.");
    }
  }
};

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__TYPES_HPP_
