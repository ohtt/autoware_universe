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

#include "../../src/filters/safety/point_cloud_collision_check/point_cloud_collision_check_filter.hpp"

#include <autoware/motion_utils/constants.hpp>
#include <autoware/motion_velocity_planner_common/utils.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{
namespace
{
constexpr double ego_nearest_dist_threshold = 3.0;
constexpr double ego_nearest_yaw_threshold = 1.046;
constexpr double decimate_trajectory_step_length = 2.0;
constexpr double goal_extended_trajectory_length = 5.0;

TrajectoryPoint make_point(const double x, const double y)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  point.longitudinal_velocity_mps = 1.0;
  return point;
}

geometry_msgs::msg::Pose make_pose(const double x, const double y)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(0.0);
  return pose;
}

std::vector<TrajectoryPoint> make_straight_trajectory(const size_t size, const double interval)
{
  std::vector<TrajectoryPoint> points;
  points.reserve(size);
  for (size_t i = 0; i < size; ++i) {
    points.push_back(make_point(interval * static_cast<double>(i), 0.0));
  }
  return points;
}

bool validate(
  const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Pose & current_pose)
{
  return validate_trajectory(
    points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    decimate_trajectory_step_length);
}

std::vector<TrajectoryPoint> decimate(
  const std::vector<TrajectoryPoint> & points, const geometry_msgs::msg::Pose & current_pose)
{
  return autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    decimate_trajectory_step_length, goal_extended_trajectory_length);
}
}  // namespace

TEST(ValidateTrajectory, AcceptsHealthyTrajectory)
{
  const auto points = make_straight_trajectory(100, 0.5);
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_TRUE(validate(points, ego));

  const auto decimated = decimate(points, ego);
  ASSERT_GE(decimated.size(), 2u);
  EXPECT_NEAR(
    autoware_utils_geometry::calc_distance2d(
      decimated.at(0).pose.position, decimated.at(1).pose.position),
    decimate_trajectory_step_length, 1e-3);
}

TEST(ValidateTrajectory, RejectsEmptyTrajectory)
{
  const std::vector<TrajectoryPoint> points;
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_FALSE(validate(points, ego));
  // findFirstNearestSegmentIndexWithSoftConstraints -> validateNonEmpty
  EXPECT_THROW(decimate(points, ego), std::invalid_argument);
}

TEST(ValidateTrajectory, RejectsSinglePointTrajectory)
{
  const auto points = make_straight_trajectory(1, 0.5);
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_FALSE(validate(points, ego));
  // motion_velocity_planner_common/utils.cpp:85 の traj_points.at(1)
  EXPECT_THROW(decimate(points, ego), std::out_of_range);
}

TEST(ValidateTrajectory, RejectsAllDuplicatedTrajectory)
{
  const std::vector<TrajectoryPoint> points(100, make_point(100.0, 200.0));
  const auto ego = make_pose(100.0, 200.0);

  EXPECT_FALSE(validate(points, ego));
  // spline_interpolation_points_2d.cpp:66 の getBaseValues
  EXPECT_THROW(decimate(points, ego), std::logic_error);
}

// 自車より手前が正常でも、自車以降が全て重複していれば間引きは失敗する。
// 軌道全体のユニーク点数で判定すると素通りしてしまうケース。
TEST(ValidateTrajectory, RejectsDuplicationOnlyAfterEgo)
{
  auto points = make_straight_trajectory(50, 0.5);
  for (size_t i = 0; i < 50; ++i) {
    points.push_back(make_point(25.0, 0.0));
  }
  const auto ego = make_pose(25.0, 0.0);

  ASSERT_EQ(
    autoware::motion_utils::findFirstNearestSegmentIndexWithSoftConstraints(
      points, ego, ego_nearest_dist_threshold, ego_nearest_yaw_threshold),
    50u);
  EXPECT_FALSE(validate(points, ego));
  EXPECT_THROW(decimate(points, ego), std::logic_error);
}

// ユニーク点が 2 点以上あっても、後方に重複があると spline の添字が範囲外になる。
TEST(ValidateTrajectory, RejectsDuplicationAtTail)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(2.0, 0.0), make_point(2.0, 0.0)};
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_FALSE(validate(points, ego));
  // spline_interpolation_points_2d.cpp:167 の getSplineInterpolatedYaw
  EXPECT_THROW(decimate(points, ego), std::out_of_range);
}

// 前方の重複は例外にはならないが、resampleTrajectory が入力をそのまま返すため
// 間引きが行われず decimate_trajectory_step_length の前提が崩れる。
TEST(ValidateTrajectory, RejectsDuplicationAtHeadThatSkipsDecimation)
{
  const std::vector<TrajectoryPoint> points{
    make_point(0.0, 0.0), make_point(0.0, 0.0), make_point(2.0, 0.0)};
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_FALSE(validate(points, ego));

  const auto decimated = decimate(points, ego);
  ASSERT_GE(decimated.size(), 2u);
  EXPECT_LT(
    autoware_utils_geometry::calc_distance2d(
      decimated.at(0).pose.position, decimated.at(1).pose.position),
    decimate_trajectory_step_length);
}

TEST(ValidateTrajectory, RejectsStepLengthBelowOverlapThreshold)
{
  const auto points = make_straight_trajectory(100, 0.5);
  const auto ego = make_pose(0.0, 0.0);

  EXPECT_FALSE(validate_trajectory(
    points, ego, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    autoware::motion_utils::overlap_threshold / 2.0));
}
}  // namespace autoware::trajectory_validator::plugin::safety
