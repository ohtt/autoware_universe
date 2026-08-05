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

#ifndef FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
#define FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "debug_marker.hpp"
#include "parameter.hpp"
#include "planner_data_lite.hpp"
#include "types.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <deque>
#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
using point_cloud_collision_check::CollisionPointWithDist;
using point_cloud_collision_check::CommonParam;
using point_cloud_collision_check::DebugData;
using point_cloud_collision_check::DetectionPolygon;
using point_cloud_collision_check::ObstacleFilteringParam;
using point_cloud_collision_check::Odometry;
using point_cloud_collision_check::PlannerData;
using point_cloud_collision_check::PointcloudSegmentationParam;
using point_cloud_collision_check::PointcloudStopCandidate;
using point_cloud_collision_check::Polygon2d;
using point_cloud_collision_check::PolygonParam;
using point_cloud_collision_check::StopObstacle;
using point_cloud_collision_check::StopObstacleClassification;
using point_cloud_collision_check::StopPlanningParam;
using point_cloud_collision_check::TrajectoryPoint;
using point_cloud_collision_check::TrajectoryPolygonCollisionCheck;

/// @brief Tells whether decimate_trajectory_points_from_ego() can decimate this candidate.
/// A diffusion generator may emit a trajectory that collapses to a point once trimmed at ego, and
/// the resampler then either throws or silently returns the input undecimated.
/// Mirrors motion_utils/resample/resample_utils.hpp validate_arguments() over the points from ego.
bool validate_trajectory(
  const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & current_pose,
  const double ego_nearest_dist_threshold, const double ego_nearest_yaw_threshold,
  const double decimate_trajectory_step_length);

/**
 * @brief PointCloudCollisionCheckFilter class - checks the trajectory against the semantic
 * segmentation point cloud produced by the perception pipeline.
 */
class PointCloudCollisionCheckFilter final : public plugin::ValidatorInterface
{
public:
  PointCloudCollisionCheckFilter() : ValidatorInterface("point_cloud_collision_check_filter") {}
  ~PointCloudCollisionCheckFilter() override = default;

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

private:
  /// @brief Tells whether the inputs needed for an evaluation are present and usable.
  /// When false, is_feasible skips the evaluation and reports feasible.
  bool is_available_data(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) const;

  /// @brief Sets the parameter-derived fields of planner_data_.
  /// The port source splits this between the PlannerData constructor and on_set_param.
  void set_planner_data_param(const validator::Params::PointCloudCollisionCheck & p);

  /// @brief Updates the topic-derived fields of planner_data_ and preprocesses the point cloud.
  /// The port source does this in the node instead of the module.
  void update_planner_data(
    const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context);

  /// @brief Extracts stop obstacles from the point cloud. Port of the point cloud path of
  /// ObstacleStopModule::plan().
  std::vector<StopObstacle> calc_obstacle_stop(
    const std::vector<TrajectoryPoint> & raw_trajectory_points, const PlannerData & planner_data);

  std::vector<StopObstacle> filter_stop_obstacle_for_point_cloud(
    const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
    const std::vector<TrajectoryPoint> & decimated_traj_points,
    const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info,
    const double x_offset_to_bumper,
    const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check);

  std::optional<double> calc_ego_forwarding_braking_distance(
    const std::vector<TrajectoryPoint> & traj_points, const Odometry & odometry) const;

  DetectionPolygon get_trajectory_polygon(
    const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
    const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
    const bool enable_to_consider_current_pose, const double time_to_convergence,
    const double decimate_trajectory_step_length) const;

  std::optional<CollisionPointWithDist> get_nearest_collision_point(
    const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
    const PlannerData::Pointcloud & point_cloud, const double x_offset_to_bumper,
    const VehicleInfo & vehicle_info) const;

  /// @brief Associates the collision point of this cycle with a tracked candidate and updates its
  /// velocity estimate. Assumes is_feasible evaluates a single candidate trajectory per cycle,
  /// because the candidate deque is shared across cycles.
  void upsert_pointcloud_stop_candidates(
    const CollisionPointWithDist & nearest_collision_point,
    const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time);

  /// @brief Substitute for ObstacleStopModule::plan_stop(), which inserts a stop point instead of
  /// answering feasible / infeasible. Reports infeasible when the nearest obstacle is closer than
  /// the distance ego needs to stop in front of it.
  /// @param[out] required_distance Ego stopping distance plus stop_margin, for the debug markers.
  bool judge_stop_feasibility(
    const std::vector<StopObstacle> & stop_obstacles, const geometry_msgs::msg::Twist & twist,
    double & required_distance) const;

  CommonParam common_param_;
  StopPlanningParam stop_planning_param_;
  std::unordered_map<StopObstacleClassification::Type, ObstacleFilteringParam>
    obstacle_filtering_params_;
  PointcloudSegmentationParam pointcloud_segmentation_param_;

  // Velocity estimation needs observations from several cycles, so the deque outlives one cycle.
  std::deque<PointcloudStopCandidate> pointcloud_stop_candidates;
  mutable std::map<PolygonParam, DetectionPolygon> trajectory_polygon_for_inside_map_;
  rclcpp::Logger logger_{rclcpp::get_logger("point_cloud_collision_check_filter")};
  // The port source takes the clock in init(); a plugin has no node, so it comes from the context.
  rclcpp::Clock::SharedPtr clock_;

  bool enable_debug_markers_{};
  PlannerData planner_data_;
  DebugData debug_data_;
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // FILTERS__SAFETY__POINT_CLOUD_COLLISION_CHECK__POINT_CLOUD_COLLISION_CHECK_FILTER_HPP_
