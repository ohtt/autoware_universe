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

#include "point_cloud_collision_check_filter.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/motion_velocity_planner_common/polygon_utils.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <rclcpp/time.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace polygon_utils = autoware::motion_velocity_planner::polygon_utils;
namespace utils = autoware::motion_velocity_planner::utils;

using point_cloud_collision_check::emit_debug_markers;
using point_cloud_collision_check::Point2d;
using point_cloud_collision_check::PointcloudPreprocessParams;
using point_cloud_collision_check::process_no_ground_pointcloud;
using point_cloud_collision_check::RSSParam;

namespace
{
// motion_velocity_obstacle_stop_module/decision_helpers.hpp:77-84
double calc_x_offset_to_bumper(const bool is_driving_forward, const VehicleInfo & vehicle_info)
{
  if (is_driving_forward) {
    return vehicle_info.max_longitudinal_offset_m;
  }
  return vehicle_info.min_longitudinal_offset_m;
}

// motion_velocity_obstacle_stop_module/decision_helpers.hpp:104-125
double calc_braking_dist_along_trajectory(
  const StopObstacleClassification::Type label, const double lon_vel, const RSSParam & rss_params)
{
  const double braking_acc = [&]() {
    if (label == StopObstacleClassification::Type::POINTCLOUD) {
      return rss_params.pointcloud_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::UNKNOWN ||
      label == StopObstacleClassification::Type::PEDESTRIAN) {
      return rss_params.no_wheel_objects_deceleration;
    }
    if (
      label == StopObstacleClassification::Type::BICYCLE ||
      label == StopObstacleClassification::Type::MOTORCYCLE) {
      return rss_params.two_wheel_objects_deceleration;
    }
    return rss_params.vehicle_objects_deceleration;
  }();
  const double error_considered_vel = std::max(lon_vel + rss_params.velocity_offset, 0.0);
  return error_considered_vel * error_considered_vel * 0.5 / -braking_acc;
}

// motion_velocity_obstacle_stop_module/decision_helpers.hpp:129-149
PolygonParam create_polygon_param(
  const ObstacleFilteringParam::TrimTrajectoryParam & trim_trajectory_param,
  const std::optional<double> ego_braking_distance,
  const ObstacleFilteringParam::LateralMarginParam & lateral_margin_param,
  const std::optional<double> object_velocity)
{
  PolygonParam p;
  if (!trim_trajectory_param.enable_trimming || !ego_braking_distance.has_value()) {
    p.trimming_length = std::nullopt;
  } else {
    p.trimming_length =
      trim_trajectory_param.min_trajectory_length +
      trim_trajectory_param.braking_distance_scale_factor * ego_braking_distance.value();
  }
  p.lateral_margin = lateral_margin_param.nominal_margin +
                     (object_velocity > lateral_margin_param.is_moving_threshold_velocity
                        ? lateral_margin_param.additional_is_moving_margin
                        : lateral_margin_param.additional_is_stop_margin);
  p.off_track_scale = lateral_margin_param.additional_wheel_off_track_scale;
  return p;
}

// motion_velocity_obstacle_stop_module/decision_helpers.hpp:40-48
double calc_minimum_distance_to_stop(
  const double initial_vel, const double max_acc, const double min_acc)
{
  if (initial_vel < 0.0) {
    return -std::pow(initial_vel, 2) / 2.0 / max_acc;
  }
  return -std::pow(initial_vel, 2) / 2.0 / min_acc;
}

}  // namespace

// Stands in for motion_velocity_planner/node.cpp:144-155 (check_with_log)
bool PointCloudCollisionCheckFilter::is_available_data(const FilterContext & context) const
{
  return context.odometry && context.acceleration && vehicle_info_ptr_ &&
         context.segmented_pointcloud && context.tf_buffer && context.clock;
}

// motion_velocity_planner_common/planner_data.cpp:239-260 and
// motion_velocity_planner/node.cpp:262-266 (set_velocity_smoother_params)
void PointCloudCollisionCheckFilter::set_planner_data_param(
  const validator::Params::PointCloudCollisionCheck & p)
{
  planner_data_.ego_nearest_dist_threshold = p.trajectory_polygon.ego_nearest_dist_threshold;
  planner_data_.ego_nearest_yaw_threshold = p.trajectory_polygon.ego_nearest_yaw_threshold;
  planner_data_.trajectory_polygon_collision_check = {
    p.trajectory_polygon.decimate_trajectory_step_length,
    p.trajectory_polygon.goal_extended_trajectory_length,
    p.trajectory_polygon.enable_to_consider_current_pose, p.trajectory_polygon.time_to_convergence};

  planner_data_.no_ground_pointcloud.preprocess_params_ = PointcloudPreprocessParams{p};

  // Stands in for motion_velocity_planner/node.cpp:262-266 (set_velocity_smoother_params).
  planner_data_.velocity_smoother_.min_decel = p.common.min_accel;
  planner_data_.velocity_smoother_.min_jerk = p.common.min_jerk;
}

bool PointCloudCollisionCheckFilter::update_planner_data(
  const std::vector<TrajectoryPoint> & raw_trajectory_points, const FilterContext & context)
{
  // motion_velocity_planner_common/planner_data.cpp:240-241
  planner_data_.vehicle_info_ = *vehicle_info_ptr_;

  // motion_velocity_planner/node.cpp:160-167
  planner_data_.current_odometry = *context.odometry;
  planner_data_.current_acceleration = *context.acceleration;

  // motion_velocity_planner/node.cpp:211-215
  const auto is_driving_forward =
    autoware::motion_utils::isDrivingForwardWithTwist(raw_trajectory_points);
  if (is_driving_forward) {
    planner_data_.is_driving_forward = is_driving_forward.value();
  }

  // motion_velocity_planner/node.cpp:176-195
  auto no_ground_pointcloud =
    process_no_ground_pointcloud(context.segmented_pointcloud, *context.tf_buffer, context.clock);
  if (!no_ground_pointcloud) {
    return false;
  }
  planner_data_.no_ground_pointcloud.preprocess_pointcloud(
    std::move(*no_ground_pointcloud), raw_trajectory_points, planner_data_.current_odometry,
    planner_data_.calculate_min_deceleration_distance(0.0).value_or(0.0),
    planner_data_.vehicle_info_, planner_data_.trajectory_polygon_collision_check,
    planner_data_.ego_nearest_dist_threshold, planner_data_.ego_nearest_yaw_threshold);
  return true;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:169-230 (plan)
std::vector<StopObstacle> PointCloudCollisionCheckFilter::calc_obstacle_stop(
  const std::vector<TrajectoryPoint> & raw_trajectory_points, const PlannerData & planner_data)
{
  const double x_offset_to_bumper =
    calc_x_offset_to_bumper(planner_data.is_driving_forward, planner_data.vehicle_info_);
  trajectory_polygon_for_inside_map_.clear();

  const auto decimated_traj_points = utils::decimate_trajectory_points_from_ego(
    raw_trajectory_points, planner_data.current_odometry.pose.pose,
    planner_data.ego_nearest_dist_threshold, planner_data.ego_nearest_yaw_threshold,
    planner_data.trajectory_polygon_collision_check.decimate_trajectory_step_length,
    stop_planning_param_.stop_margin);

  return filter_stop_obstacle_for_point_cloud(
    planner_data.current_odometry, raw_trajectory_points, decimated_traj_points,
    planner_data.no_ground_pointcloud, planner_data.vehicle_info_, x_offset_to_bumper,
    planner_data.trajectory_polygon_collision_check);
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:463-565
std::vector<StopObstacle> PointCloudCollisionCheckFilter::filter_stop_obstacle_for_point_cloud(
  const Odometry & odometry, const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & decimated_traj_points,
  const PlannerData::Pointcloud & point_cloud, const VehicleInfo & vehicle_info,
  const double x_offset_to_bumper,
  const TrajectoryPolygonCollisionCheck & trajectory_polygon_collision_check)
{
  const auto & filtering_param =
    obstacle_filtering_params_.at(StopObstacleClassification::Type::POINTCLOUD);

  if (!filtering_param.check_inside) {
    return std::vector<StopObstacle>{};
  }

  if (
    point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin <
    filtering_param.lateral_margin.nominal_margin) {
    RCLCPP_WARN_ONCE(
      logger_,
      "pointcloud_preprocessing lateral margin (%f) is smaller than the obstacle_filtering nominal "
      "margin (%f), so obstacles inside the detection corridor may already be cropped away",
      point_cloud.preprocess_params_.filter_by_trajectory_polygon.lateral_margin,
      filtering_param.lateral_margin.nominal_margin);
  }

  const auto & tp = trajectory_polygon_collision_check;
  const auto polygon_param = create_polygon_param(
    filtering_param.trim_trajectory, calc_ego_forwarding_braking_distance(traj_points, odometry),
    filtering_param.lateral_margin, std::nullopt);
  const auto detection_polygon_with_lat_margin = get_trajectory_polygon(
    decimated_traj_points, vehicle_info, odometry.pose.pose, polygon_param,
    tp.enable_to_consider_current_pose, tp.time_to_convergence, tp.decimate_trajectory_step_length);

  const auto nearest_collision_point = get_nearest_collision_point(
    detection_polygon_with_lat_margin.traj_points, detection_polygon_with_lat_margin.polygons,
    point_cloud, x_offset_to_bumper, vehicle_info);

  const auto latest_point_cloud_time = rclcpp::Time(
    static_cast<int64_t>(point_cloud.pointcloud.header.stamp * static_cast<uint32_t>(1e3)),
    RCL_ROS_TIME);
  if (nearest_collision_point) {
    upsert_pointcloud_stop_candidates(
      nearest_collision_point.value(), traj_points, latest_point_cloud_time);
  }

  while (
    !pointcloud_stop_candidates.empty() &&
    (latest_point_cloud_time - pointcloud_stop_candidates.front().latest_collision_pointcloud_time)
        .seconds() > filtering_param.stop_obstacle_hold_time_threshold) {
    pointcloud_stop_candidates.pop_front();
  }

  std::vector<StopObstacle> stop_obstacles;
  for (const auto & stop_candidate : pointcloud_stop_candidates) {
    if (!stop_candidate.vel_lpf.getValue().has_value()) {
      continue;
    }

    const double time_delay =
      (clock_->now() - stop_candidate.latest_collision_pointcloud_time).seconds();
    const double time_compensated_dist_to_collide =
      stop_candidate.latest_collision_point.dist_to_collide +
      *stop_candidate.vel_lpf.getValue() * time_delay;

    const bool use_estimated_velocity =
      pointcloud_segmentation_param_.velocity_estimation.use_estimated_velocity;
    if (
      !use_estimated_velocity ||
      *stop_candidate.vel_lpf.getValue() <
        stop_planning_param_.obstacle_velocity_threshold_enter_fixed_stop) {
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param);
    } else if (stop_planning_param_.rss_params.use_rss_stop) {
      const auto braking_dist = calc_braking_dist_along_trajectory(
        StopObstacleClassification::Type::POINTCLOUD, *stop_candidate.vel_lpf.getValue(),
        stop_planning_param_.rss_params);
      stop_obstacles.emplace_back(
        stop_candidate.latest_collision_pointcloud_time,
        StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD},
        stop_candidate.vel_lpf.getValue().value(), stop_candidate.latest_collision_point.point,
        time_compensated_dist_to_collide, polygon_param, braking_dist);
      RCLCPP_DEBUG(
        logger_,
        "|_PC_| total_dist: %2.5f, raw_dist: %2.5f, time_compensated dist: %2.5f, "
        "braking_dist: %2.5f",
        (time_compensated_dist_to_collide + braking_dist),
        (stop_candidate.latest_collision_point.dist_to_collide), time_compensated_dist_to_collide,
        braking_dist);
    }
  }

  return stop_obstacles;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:232-241
std::optional<double> PointCloudCollisionCheckFilter::calc_ego_forwarding_braking_distance(
  const std::vector<TrajectoryPoint> & traj_points, const Odometry & odometry) const
{
  if (traj_points.empty() || autoware::motion_utils::isDrivingForward(traj_points) != true) {
    return std::nullopt;
  }
  return autoware::motion_utils::calcDecelDistWithJerkAndAccConstraints(
    odometry.twist.twist.linear.x, 0.0, common_param_.max_accel, common_param_.min_accel,
    common_param_.max_jerk, common_param_.min_jerk);
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:1246-1268
DetectionPolygon PointCloudCollisionCheckFilter::get_trajectory_polygon(
  const std::vector<TrajectoryPoint> & decimated_traj_points, const VehicleInfo & vehicle_info,
  const geometry_msgs::msg::Pose & current_ego_pose, const PolygonParam & polygon_param,
  const bool enable_to_consider_current_pose, const double time_to_convergence,
  const double decimate_trajectory_step_length) const
{
  if (trajectory_polygon_for_inside_map_.count(polygon_param) == 0) {
    auto cropped_traj_points =
      polygon_param.trimming_length.has_value()
        ? autoware::motion_utils::cropForwardPoints(
            decimated_traj_points, decimated_traj_points.front().pose.position, 0,
            polygon_param.trimming_length.value())
        : decimated_traj_points;

    auto traj_polys = polygon_utils::create_one_step_polygons(
      cropped_traj_points, vehicle_info, current_ego_pose, polygon_param.lateral_margin,
      enable_to_consider_current_pose, time_to_convergence, decimate_trajectory_step_length,
      polygon_param.off_track_scale);
    trajectory_polygon_for_inside_map_.emplace(
      polygon_param, DetectionPolygon{std::move(cropped_traj_points), std::move(traj_polys)});
  }
  return trajectory_polygon_for_inside_map_.at(polygon_param);
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:243-320
std::optional<CollisionPointWithDist> PointCloudCollisionCheckFilter::get_nearest_collision_point(
  const std::vector<TrajectoryPoint> & traj_points, const std::vector<Polygon2d> & traj_polygons,
  const PlannerData::Pointcloud & point_cloud, const double x_offset_to_bumper,
  const VehicleInfo & vehicle_info) const
{
  if (traj_points.size() != traj_polygons.size()) {
    RCLCPP_ERROR(
      logger_, "The size of trajectory points and polygons do not match: %zu vs %zu",
      traj_points.size(), traj_polygons.size());
    return std::nullopt;
  }

  if (point_cloud.pointcloud.empty()) {
    return std::nullopt;
  }

  const auto & clusters = point_cloud.get_cluster_indices();
  const auto & pointcloud_ptr = point_cloud.get_filtered_pointcloud_ptr();

  const auto & height_margin = pointcloud_segmentation_param_.height_margin;
  std::vector<geometry_msgs::msg::Point> collision_geom_points{};
  for (size_t traj_index = 0; traj_index < traj_points.size(); ++traj_index) {
    const double rough_dist_th =
      static_cast<double>(boost::geometry::perimeter(traj_polygons.at(traj_index))) * 0.5;
    const double traj_height = traj_points.at(traj_index).pose.position.z;

    for (const auto & cluster : clusters) {
      for (const auto & point_index : cluster.indices) {
        const auto obstacle_point = autoware::motion_velocity_planner::utils::to_geometry_point(
          pointcloud_ptr->at(point_index));
        if (
          obstacle_point.z - traj_height < -height_margin.margin_from_bottom ||
          obstacle_point.z - traj_height >
            vehicle_info.max_height_offset_m + height_margin.margin_from_top) {
          continue;
        }
        const double dist_from_base_link =
          autoware_utils_geometry::calc_distance2d(traj_points.at(traj_index).pose, obstacle_point);
        if (dist_from_base_link > rough_dist_th) {
          continue;
        }
        Point2d obstacle_point_2d{obstacle_point.x, obstacle_point.y};
        if (boost::geometry::within(obstacle_point_2d, traj_polygons.at(traj_index))) {
          collision_geom_points.push_back(obstacle_point);
        }
      }
    }
    if (collision_geom_points.empty()) {
      continue;
    }

    const auto bumper_pose = autoware_utils_geometry::calc_offset_pose(
      traj_points.at(traj_index).pose, x_offset_to_bumper, 0.0, 0.0);

    std::optional<double> max_collision_length = std::nullopt;
    std::optional<geometry_msgs::msg::Point> max_collision_point = std::nullopt;

    for (const auto & point : collision_geom_points) {
      const double dist_from_bumper =
        std::abs(autoware_utils_geometry::inverse_transform_point(point, bumper_pose).x);
      if (
        !max_collision_length.has_value() ||
        dist_from_bumper > max_collision_length.value_or(-1.0)) {
        max_collision_length = dist_from_bumper;
        max_collision_point = point;
      }
    }

    return CollisionPointWithDist{
      max_collision_point.value_or(geometry_msgs::msg::Point{}),
      autoware::motion_utils::calcSignedArcLength(traj_points, 0, traj_index) -
        max_collision_length.value_or(-1.0)};
  }
  return std::nullopt;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:405-461
void PointCloudCollisionCheckFilter::upsert_pointcloud_stop_candidates(
  const CollisionPointWithDist & nearest_collision_point,
  const std::vector<TrajectoryPoint> & traj_points, rclcpp::Time latest_point_cloud_time)
{
  const auto & vel_params = pointcloud_segmentation_param_.velocity_estimation;
  const auto & assoc_params = pointcloud_segmentation_param_.time_series_association;

  for (auto stop_candidate = pointcloud_stop_candidates.rbegin();
       stop_candidate != pointcloud_stop_candidates.rend(); ++stop_candidate) {
    const double time_since_latest_collision =
      (latest_point_cloud_time - stop_candidate->latest_collision_pointcloud_time).seconds();
    // The same point cloud frame reaching this candidate again carries no new displacement.
    if (time_since_latest_collision < 0.05) {
      return;
    }
    const double longitudinal_displacement = autoware::motion_utils::calcSignedArcLength(
      traj_points, stop_candidate->latest_collision_point.point, nearest_collision_point.point);

    if (
      time_since_latest_collision < assoc_params.max_time_diff &&
      -assoc_params.position_diff + assoc_params.min_velocity * time_since_latest_collision <
        longitudinal_displacement &&
      longitudinal_displacement <
        assoc_params.position_diff + assoc_params.max_velocity * time_since_latest_collision) {
      const double clamped_vel = std::clamp(
        longitudinal_displacement / time_since_latest_collision, vel_params.min_clamp_velocity,
        vel_params.max_clamp_velocity);
      if (!stop_candidate->vel_lpf.getValue().has_value()) {
        auto & vel_vec = stop_candidate->initial_velocities;
        vel_vec.push_back(clamped_vel);
        if (vel_vec.size() >= vel_params.required_velocity_count) {
          stop_candidate->vel_lpf.reset(
            std::accumulate(vel_vec.begin(), vel_vec.end(), 0.0) /
            static_cast<double>(vel_vec.size()));
        }
      } else {
        stop_candidate->vel_lpf.filter(clamped_vel);
      }
      stop_candidate->latest_collision_point = nearest_collision_point;
      stop_candidate->latest_collision_pointcloud_time = latest_point_cloud_time;

      // Keep the deque ordered by observation time so that the hold expiry can pop from the front.
      std::sort(
        pointcloud_stop_candidates.begin(), pointcloud_stop_candidates.end(),
        [](const PointcloudStopCandidate & a, const PointcloudStopCandidate & b) {
          return a.latest_collision_pointcloud_time < b.latest_collision_pointcloud_time;
        });

      return;
    }
  }

  PointcloudStopCandidate new_stop_candidate;
  new_stop_candidate.latest_collision_point = nearest_collision_point;
  new_stop_candidate.latest_collision_pointcloud_time = latest_point_cloud_time;
  new_stop_candidate.vel_lpf.setGain(vel_params.lpf_gain);
  pointcloud_stop_candidates.push_back(new_stop_candidate);
}

// Stands in for motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:815-934 (plan_stop)
bool PointCloudCollisionCheckFilter::judge_stop_feasibility(
  const std::vector<StopObstacle> & stop_obstacles, const geometry_msgs::msg::Twist & twist,
  double & required_distance) const
{
  std::optional<double> nearest_dist_to_collide;
  for (const auto & stop_obstacle : stop_obstacles) {
    // Under RSS the obstacle keeps moving away while ego brakes, so its own braking distance is
    // added to the distance ego may cover.
    const double dist_to_collide =
      stop_obstacle.dist_to_collide_on_decimated_traj + stop_obstacle.braking_dist.value_or(0.0);
    if (!nearest_dist_to_collide.has_value() || dist_to_collide < *nearest_dist_to_collide) {
      nearest_dist_to_collide = dist_to_collide;
    }
  }

  required_distance =
    stop_planning_param_.stop_margin +
    calc_minimum_distance_to_stop(twist.linear.x, common_param_.max_accel, common_param_.min_accel);

  return !nearest_dist_to_collide.has_value() || *nearest_dist_to_collide >= required_distance;
}

// Stands in for motion_velocity_planner/node.cpp:297-353 (on_trajectory)
PointCloudCollisionCheckFilter::result_t PointCloudCollisionCheckFilter::is_feasible(
  const CandidateTrajectory & candidate_trajectory, const FilterContext & context)
{
  // memo: assume trajectory_selector subscribes "/perception/obstacle_segmentation/pointcloud" or
  // "/perception/segmented/pointcloud" that was published from ptv3 node

  if (!is_available_data(context)) {
    return ValidationResult{};
  }
  clock_ = context.clock;

  // Without the preprocessed point cloud there is nothing to collide against, and the debug markers
  // would read a stale corridor from the previous cycle.
  if (!update_planner_data(candidate_trajectory.points, context)) {
    return ValidationResult{};
  }

  const auto stop_obstacles = calc_obstacle_stop(candidate_trajectory.points, planner_data_);

  ValidationResult result{};
  double required_distance = 0.0;
  result.is_feasible =
    judge_stop_feasibility(stop_obstacles, context.odometry->twist.twist, required_distance);

  if (enable_debug_markers_) {
    emit_debug_markers(
      debug_markers_, debug_data_, planner_data_, stop_obstacles, required_distance,
      result.is_feasible, candidate_trajectory.generator_id.uuid,
      rclcpp::Time{context.odometry->header.stamp});
  }

  return result;
}

// motion_velocity_obstacle_stop_module/obstacle_stop_module.cpp:116-121 (the parameter part of
// init)
void PointCloudCollisionCheckFilter::update_parameters(const validator::Params & params)
{
  const auto & p = params.point_cloud_collision_check;

  common_param_ = CommonParam{p};
  stop_planning_param_ = StopPlanningParam{p};
  obstacle_filtering_params_ = {
    {StopObstacleClassification::Type::POINTCLOUD, ObstacleFilteringParam{p}}};
  pointcloud_segmentation_param_ = PointcloudSegmentationParam{p};
  set_planner_data_param(p);

  enable_debug_markers_ = p.debug.enable_markers;
}
}  // namespace autoware::trajectory_validator::plugin::safety

#include <pluginlib/class_list_macros.hpp>
namespace safety = autoware::trajectory_validator::plugin::safety;

PLUGINLIB_EXPORT_CLASS(
  safety::PointCloudCollisionCheckFilter,
  autoware::trajectory_validator::plugin::ValidatorInterface)
