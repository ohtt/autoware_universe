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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_

#include "planner_data_lite.hpp"
#include "types.hpp"

#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check
{
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// @brief Debug snapshot of one candidate trajectory, all in the map frame. Built only from what
/// the filter itself can see - the preprocessed point cloud, the stop obstacles and the verdict.
struct DebugData
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pointcloud_ptr;
  std::optional<geometry_msgs::msg::Point> nearest_collision_point;
  std::optional<double> dist_to_collide;
  double required_distance{};
  bool is_feasible{true};
  geometry_msgs::msg::Point ego_position;

  /// @brief A tracked obstacle. Only candidates whose velocity estimate has settled become stop
  /// obstacles, so unsettled candidates never appear here.
  struct Track
  {
    geometry_msgs::msg::Point point;
    // Longitudinal velocity along the candidate trajectory. It carries no direction, so it is drawn
    // as a vertical bar rather than an arrow along the trajectory.
    double velocity{0.0};
    bool settled{false};
  };
  std::vector<Track> tracks;
  std::string status_text;
  // 0 = safe (green) / 1 = confirming (yellow) / 2 = stop required (red).
  int status_level{0};
  std::array<float, 3> generator_color{};
};

inline std::int32_t next_marker_id(const MarkerArray & markers)
{
  return markers.markers.empty() ? 0 : markers.markers.back().id + 1;
}

inline std_msgs::msg::ColorRGBA make_color(
  const float r, const float g, const float b, const float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

inline Marker base_marker(
  const std::string & ns, const std::int32_t id, const std::int32_t type,
  const rclcpp::Time & stamp)
{
  Marker m;
  m.header.frame_id = "map";
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  // take_debug_markers() prepends a DELETEALL every cycle, so the markers need no lifetime.
  m.lifetime = rclcpp::Duration::from_seconds(0.0);
  return m;
}

// Exactly one marker with this namespace suffix is emitted per candidate, so counting them gives
// the index of the candidate currently being evaluated.
constexpr const char * candidate_marker_ns_suffix = "/feasibility";

inline int count_candidate_markers(const MarkerArray & markers)
{
  const std::string suffix{candidate_marker_ns_suffix};
  return static_cast<int>(
    std::count_if(markers.markers.begin(), markers.markers.end(), [&suffix](const Marker & marker) {
      return marker.ns.size() >= suffix.size() &&
             marker.ns.compare(marker.ns.size() - suffix.size(), suffix.size(), suffix) == 0;
    }));
}

inline geometry_msgs::msg::Point make_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/// @brief Hashes the 16 generator id bytes with FNV-1a so that each generator keeps a stable color.
inline std::array<float, 3> generator_color_from_uuid(const std::array<std::uint8_t, 16> & uuid)
{
  std::uint32_t hash = 2166136261U;
  for (const auto byte : uuid) {
    hash = (hash ^ byte) * 16777619U;
  }
  static const std::array<std::array<float, 3>, 6> palette = {
    {{0.1F, 0.6F, 1.0F},
     {1.0F, 0.5F, 0.1F},
     {0.4F, 1.0F, 0.4F},
     {1.0F, 0.4F, 0.8F},
     {0.9F, 0.9F, 0.2F},
     {0.6F, 0.4F, 1.0F}}};
  return palette.at(hash % palette.size());
}

inline void add_candidate_debug_markers(
  MarkerArray & markers, const DebugData & debug_data, const int candidate_index,
  const rclcpp::Time & stamp)
{
  const std::string ns_prefix = std::to_string(candidate_index);

  if (debug_data.filtered_pointcloud_ptr && !debug_data.filtered_pointcloud_ptr->empty()) {
    auto m = base_marker(ns_prefix + "/clusters", next_marker_id(markers), Marker::POINTS, stamp);
    m.scale.x = 0.2;
    m.scale.y = 0.2;
    m.color = make_color(
      debug_data.generator_color.at(0), debug_data.generator_color.at(1),
      debug_data.generator_color.at(2), 0.9);
    for (const auto & p : debug_data.filtered_pointcloud_ptr->points) {
      m.points.push_back(autoware::motion_velocity_planner::utils::to_geometry_point(p));
    }
    markers.markers.push_back(m);
  }

  if (debug_data.nearest_collision_point) {
    auto m =
      base_marker(ns_prefix + "/collision_point", next_marker_id(markers), Marker::SPHERE, stamp);
    m.pose.position = *debug_data.nearest_collision_point;
    m.scale.x = m.scale.y = m.scale.z = 0.6;
    m.color = make_color(1.0, 0.1, 0.1, 0.9);
    markers.markers.push_back(m);
  }

  auto text = base_marker(
    ns_prefix + candidate_marker_ns_suffix, next_marker_id(markers), Marker::TEXT_VIEW_FACING,
    stamp);
  if (debug_data.nearest_collision_point) {
    text.pose.position = *debug_data.nearest_collision_point;
    text.pose.position.z += 1.0;
  } else {
    // Stack the per-candidate text above ego so that candidates without an obstacle stay readable.
    text.pose.position = debug_data.ego_position;
    text.pose.position.z += 1.5 + 0.7 * static_cast<double>(candidate_index);
  }
  text.scale.z = 0.6;
  text.color =
    debug_data.is_feasible ? make_color(0.2, 1.0, 0.3, 1.0) : make_color(1.0, 0.1, 0.1, 1.0);
  std::array<char, 160> buf{};
  if (debug_data.dist_to_collide) {
    std::snprintf(
      buf.data(), buf.size(), "cand%d: %s  dist=%.2f req=%.2f", candidate_index,
      debug_data.is_feasible ? "SAFE" : "STOP REQUIRED", *debug_data.dist_to_collide,
      debug_data.required_distance);
  } else if (debug_data.nearest_collision_point) {
    std::snprintf(
      buf.data(), buf.size(), "cand%d: SAFE (obstacle detected, confirming...)", candidate_index);
  } else {
    std::snprintf(buf.data(), buf.size(), "cand%d: SAFE (clear)", candidate_index);
  }
  text.text = buf.data();
  markers.markers.push_back(text);
}

inline void add_cycle_debug_markers(
  MarkerArray & markers, const DebugData & debug_data, const rclcpp::Time & stamp)
{
  if (!debug_data.tracks.empty()) {
    auto points =
      base_marker("tracking/points", next_marker_id(markers), Marker::SPHERE_LIST, stamp);
    points.scale.x = points.scale.y = points.scale.z = 0.4;
    points.color = make_color(1.0, 1.0, 1.0, 0.8);
    for (const auto & track : debug_data.tracks) {
      points.points.push_back(track.point);
    }
    markers.markers.push_back(points);

    for (const auto & track : debug_data.tracks) {
      auto arrow = base_marker("tracking/velocity", next_marker_id(markers), Marker::ARROW, stamp);
      arrow.scale.x = 0.1;
      arrow.scale.y = 0.2;
      arrow.scale.z = 0.2;
      arrow.color = track.settled ? make_color(1.0, 1.0, 0.2, 0.9) : make_color(0.6, 0.6, 0.6, 0.5);
      arrow.points.push_back(track.point);
      arrow.points.push_back(
        make_point(track.point.x, track.point.y, track.point.z + track.velocity));
      markers.markers.push_back(arrow);
    }
  }

  auto banner = base_marker("status", next_marker_id(markers), Marker::TEXT_VIEW_FACING, stamp);
  banner.pose.position = debug_data.ego_position;
  banner.pose.position.z += 3.0;
  banner.scale.z = 0.9;
  switch (debug_data.status_level) {
    case 2:
      banner.color = make_color(1.0, 0.1, 0.1, 1.0);
      break;
    case 1:
      banner.color = make_color(1.0, 0.9, 0.2, 1.0);
      break;
    default:
      banner.color = make_color(0.2, 1.0, 0.3, 1.0);
      break;
  }
  banner.text =
    debug_data.status_text.empty() ? std::string{"PCC: monitoring"} : debug_data.status_text;
  markers.markers.push_back(banner);
}

inline DebugData make_debug_data(
  const PlannerData & planner_data, const std::vector<StopObstacle> & stop_obstacles,
  const double required_distance, const bool is_feasible)
{
  DebugData debug;
  debug.filtered_pointcloud_ptr = planner_data.no_ground_pointcloud.get_filtered_pointcloud_ptr();
  debug.ego_position = planner_data.current_odometry.pose.pose.position;
  debug.required_distance = required_distance;
  debug.is_feasible = is_feasible;

  for (const auto & stop_obstacle : stop_obstacles) {
    DebugData::Track track;
    track.point = stop_obstacle.collision_point;
    track.velocity = stop_obstacle.velocity;
    track.settled = true;
    debug.tracks.push_back(track);
  }

  const auto nearest = std::min_element(
    stop_obstacles.begin(), stop_obstacles.end(),
    [](const StopObstacle & a, const StopObstacle & b) {
      const double dist_a = a.dist_to_collide_on_decimated_traj + a.braking_dist.value_or(0.0);
      const double dist_b = b.dist_to_collide_on_decimated_traj + b.braking_dist.value_or(0.0);
      return dist_a < dist_b;
    });
  if (nearest != stop_obstacles.end()) {
    debug.nearest_collision_point = nearest->collision_point;
    debug.dist_to_collide =
      nearest->dist_to_collide_on_decimated_traj + nearest->braking_dist.value_or(0.0);
  }
  return debug;
}

inline void emit_debug_markers(
  MarkerArray & markers, DebugData & debug, const PlannerData & planner_data,
  const std::vector<StopObstacle> & stop_obstacles, const double required_distance,
  const bool is_feasible, const std::array<std::uint8_t, 16> & generator_uuid,
  const rclcpp::Time & stamp)
{
  debug = make_debug_data(planner_data, stop_obstacles, required_distance, is_feasible);
  debug.generator_color = generator_color_from_uuid(generator_uuid);

  // take_debug_markers() drains the array every cycle, so an empty array means the first candidate.
  const bool first_candidate = markers.markers.empty();
  add_candidate_debug_markers(markers, debug, count_candidate_markers(markers), stamp);

  if (!first_candidate) {
    return;
  }
  debug.status_level = debug.is_feasible ? 0 : 2;
  debug.status_text = std::string{"PCC: "} + (debug.is_feasible ? "SAFE" : "STOP REQUIRED") +
                      " | tracked obstacles:" + std::to_string(debug.tracks.size());
  add_cycle_debug_markers(markers, debug, stamp);
}

}  // namespace autoware::trajectory_validator::plugin::safety::point_cloud_collision_check

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__DEBUG_MARKER_HPP_
