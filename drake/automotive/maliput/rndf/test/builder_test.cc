#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <ignition/math/Vector3.hh>

#include <ignition/rndf/UniqueId.hh>

#include "drake/common/drake_throw.h"
#include "drake/automotive/maliput/rndf/builder.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-2;

#define EXPECT_GEO_NEAR(actual, expected, tolerance)         \
do {                                                       \
  const api::GeoPosition _actual(actual);                  \
  const api::GeoPosition _expected(expected);               \
  const double _tolerance = (tolerance);                   \
  EXPECT_NEAR(_actual.x(), _expected.x(), _tolerance);         \
  EXPECT_NEAR(_actual.y(), _expected.y(), _tolerance);         \
  EXPECT_NEAR(_actual.z(), _expected.z(), _tolerance);         \
} while (0)

#define EXPECT_BOUNDS_NEAR(actual, expected, tolerance)      \
do {                                                         \
  const api::RBounds _actual(actual);                        \
  const api::RBounds _expected(expected);                    \
  const double _tolerance = (tolerance);                     \
  EXPECT_NEAR(_actual.r_min, _expected.r_min, _tolerance);   \
  EXPECT_NEAR(_actual.r_max, _expected.r_max, _tolerance);   \
} while (0)

int FindJunction(const api::RoadGeometry &road_geometry,
  const std::string &junction_name) {
  for (int i = 0; i < road_geometry.num_junctions(); i++) {
    if (road_geometry.junction(i)->id().id == junction_name) {
      return i;
    }
  }
  return -1;
}

// #  * 1.1.1       #
// #   \            #
// #    \           #
// #     * 1.1.2    #
// #    /           #
// #   /            #
// #  * 1.1.3       #
// #   \            #
// #    \           #
// #     * 1.1.4    #
GTEST_TEST(RNDFBuilder, ZigZagLane) {
  const double width = 5.;

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    0.01,
    0.01 * M_PI);

  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(0., 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(10.0, -10.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 3),
    ignition::math::Vector3d(0.0, -20.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 4),
    ignition::math::Vector3d(10.0, -30.0, 0.0)));
  Connection l1(std::to_string(1),
    waypoints,
    width);

  std::vector<Connection> connected_lanes = {l1};

  auto bounding_box = std::make_tuple<ignition::math::Vector3d,
    ignition::math::Vector3d> (ignition::math::Vector3d(0., -30., 0.),
      ignition::math::Vector3d(10., 0., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"ZigZagLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check the junctions, segments and lanes
  EXPECT_EQ(road_geometry->num_junctions(), 3);
  EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id,
    std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_1-1_1_2"));

  EXPECT_EQ(road_geometry->junction(1)->id().id, std::string("j:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id,
    std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_2-1_1_3"));

  EXPECT_EQ(road_geometry->junction(2)->id().id, std::string("j:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id,
    std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_3-1_1_4"));

  // Check the branchpoints
  EXPECT_EQ(road_geometry->num_branch_points(), 4);
  EXPECT_EQ(road_geometry->branch_point(0)->id().id,
    std::string("bp:") + std::to_string(0));
  EXPECT_EQ(road_geometry->branch_point(0)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(0)->GetBSide()->size(), 0);

  EXPECT_EQ(road_geometry->branch_point(1)->id().id,
    std::string("bp:") + std::to_string(1));
  EXPECT_EQ(road_geometry->branch_point(1)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(1)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(2)->id().id,
    std::string("bp:") + std::to_string(2));
  EXPECT_EQ(road_geometry->branch_point(2)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(2)->GetBSide()->size(), 1);

  EXPECT_EQ(road_geometry->branch_point(3)->id().id,
    std::string("bp:") + std::to_string(3));
  EXPECT_EQ(road_geometry->branch_point(3)->GetASide()->size(), 1);
  EXPECT_EQ(road_geometry->branch_point(3)->GetBSide()->size(), 0);

  // Check the brach point assigment regarding the lanes
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(0));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(1));

  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(1));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(2));

  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kStart),
    road_geometry->branch_point(2));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->
      GetBranchPoint(api::LaneEnd::kFinish),
    road_geometry->branch_point(3));
}


// #         1.1.1      1.1.2       1.1.3           #
// #         *----------*-----------*               #
// # 1.2.1   1.2.2  1.2.3    1.2.4                  #
// # *-------*------*--------*                      #
// #             1.3.2                      1.3.1   #
// #             *--------------------------*       #
GTEST_TEST(RNDFBuilder, MultilaneLane) {
  const double width = 5.;
  const api::RBounds single_lane_bounds(-width / 2.0, width / 2.0);
  const api::RBounds two_lane_bounds_left(-width / 2.0, width / 2.0 + 10.0);
  const api::RBounds two_lane_bounds_right(-width / 2.0 - 10.0, width / 2.0);

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    0.01,
    0.01 * M_PI);

  std::vector<Connection> connected_lanes;

  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(0., 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(10.0, 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(15.0, 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 4),
    ignition::math::Vector3d(25.0, 0.0, 0.0)));
  Connection l(std::to_string(1),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }

  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(10., 10.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(20.0, 10.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 3),
    ignition::math::Vector3d(30.0, 10.0, 0.0)));
  Connection l(std::to_string(1),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }

  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 1),
    ignition::math::Vector3d(40., -10.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 3, 2),
    ignition::math::Vector3d(5.0, -10.0, 0.0)));
  Connection l(std::to_string(1),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }

  auto bounding_box = std::make_tuple<ignition::math::Vector3d,
    ignition::math::Vector3d> (ignition::math::Vector3d(0., -10., 0.),
      ignition::math::Vector3d(40., 10., 0.));
  builder->SetBoundingBox(bounding_box);
  builder->CreateSegmentConnections(1, &connected_lanes);

  auto road_geometry = builder->Build({"MultilaneLane"});
  EXPECT_NE(road_geometry, nullptr);

  // Check road creation, naming, which lane is at both sides and bounds.
  EXPECT_EQ(road_geometry->num_junctions(), 6);
  EXPECT_EQ(road_geometry->junction(0)->id().id, std::string("j:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->id().id,
    std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->id().id,
    std::string("l:1_2_1-1_2_2"));
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(0)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(0)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(0)->segment(0)->lane(0)->driveable_bounds(0),
    single_lane_bounds,
    kLinearTolerance);

  EXPECT_EQ(road_geometry->junction(1)->id().id, std::string("j:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->id().id,
    std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->id().id,
    std::string("l:1_2_2-1_2_3"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->id().id,
    std::string("l:1_1_1-1_1_5"));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->to_left(),
    road_geometry->junction(1)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(1)->segment(0)->lane(1)->to_right(),
    road_geometry->junction(1)->segment(0)->lane(0));
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(1)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(1)->segment(0)->lane(1)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(1)->segment(0)->lane(0)->driveable_bounds(0),
    two_lane_bounds_left,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(1)->segment(0)->lane(1)->driveable_bounds(0),
    two_lane_bounds_right,
    kLinearTolerance);

  EXPECT_EQ(road_geometry->junction(2)->id().id, std::string("j:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->id().id,
    std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->id().id,
    std::string("l:1_2_3-1_2_5"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->id().id,
    std::string("l:1_1_5-1_1_2"));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->to_left(),
    road_geometry->junction(2)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(2)->segment(0)->lane(1)->to_right(),
    road_geometry->junction(2)->segment(0)->lane(0));
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(2)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(2)->segment(0)->lane(1)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(2)->segment(0)->lane(0)->driveable_bounds(0),
    two_lane_bounds_left,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(2)->segment(0)->lane(1)->driveable_bounds(0),
    two_lane_bounds_right,
    kLinearTolerance);

  EXPECT_EQ(road_geometry->junction(3)->id().id, std::string("j:1-0-3"));
  EXPECT_EQ(road_geometry->junction(3)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->id().id,
    std::string("s:1-0-3"));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->num_lanes(), 2);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->id().id,
    std::string("l:1_2_5-1_2_4"));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->id().id,
    std::string("l:1_1_2-1_1_6"));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->to_left(),
    road_geometry->junction(3)->segment(0)->lane(1));
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(3)->segment(0)->lane(1)->to_right(),
    road_geometry->junction(3)->segment(0)->lane(0));
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(3)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(3)->segment(0)->lane(1)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(3)->segment(0)->lane(0)->driveable_bounds(0),
    two_lane_bounds_left,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(3)->segment(0)->lane(1)->driveable_bounds(0),
    two_lane_bounds_right,
    kLinearTolerance);

  EXPECT_EQ(road_geometry->junction(4)->id().id, std::string("j:1-0-4"));
  EXPECT_EQ(road_geometry->junction(4)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->id().id,
    std::string("s:1-0-4"));
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->id().id,
    std::string("l:1_1_6-1_1_3"));
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(4)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(4)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(4)->segment(0)->lane(0)->driveable_bounds(0),
    single_lane_bounds,
    kLinearTolerance);

  EXPECT_EQ(road_geometry->junction(5)->id().id, std::string("j:1-1-0"));
  EXPECT_EQ(road_geometry->junction(5)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->id().id,
    std::string("s:1-1-0"));
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->num_lanes(), 1);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->id().id,
    std::string("l:1_3_1-1_3_2"));
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->to_left(),
    nullptr);
  EXPECT_EQ(road_geometry->junction(5)->segment(0)->lane(0)->to_right(),
    nullptr);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(5)->segment(0)->lane(0)->lane_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
  EXPECT_BOUNDS_NEAR(
    road_geometry->junction(5)->segment(0)->lane(0)->driveable_bounds(0),
    single_lane_bounds,
    kLinearTolerance);
}


// #              1.1.1      1.2.3                #
// #                    *   *                     #
// #                    |   |                     #
// #              1.1.2 |   |                     #
// #   2.2.2    2.2.1   *   *                     #
// #  *--------------*/ |  /| 1.2.2               #
// #   2.1.1    2.1.2   | / |           2.1.3     #
// #  *---------------*/---------------*          #
// #                   \|   |                     #
// #              1.1.3 *   |                     #
// #                    |   |                     #
// #                    |   |                     #
// #              1.1.4 *   * 1.2.1               #
GTEST_TEST(RNDFBuilder, MultilaneLaneCross) {
  const double width = 5.;

  std::unique_ptr<Builder> builder = std::make_unique<Builder>(
    0.01,
    0.01 * M_PI);
  auto bounding_box = std::make_tuple<ignition::math::Vector3d,
    ignition::math::Vector3d> (ignition::math::Vector3d(0., 0., 0.),
      ignition::math::Vector3d(40., 50., 0.));
  builder->SetBoundingBox(bounding_box);

  std::vector<Connection> connected_lanes;

  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 1),
    ignition::math::Vector3d(20., 50.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 2),
    ignition::math::Vector3d(20.0, 40.0, 0.0),
    false, true));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 3),
    ignition::math::Vector3d(20.0, 10.0, 0.0),
    true, false));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 1, 4),
    ignition::math::Vector3d(20.0, 0.0, 0.0)));
  Connection l(std::to_string(1),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }
  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 1),
    ignition::math::Vector3d(30., 0.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 2),
    ignition::math::Vector3d(30.0, 30.0, 0.0),
    true, false));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(1, 2, 3),
    ignition::math::Vector3d(30.0, 50.0, 0.0)));
  Connection l(std::to_string(1),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }
  builder->CreateSegmentConnections(1, &connected_lanes);

  connected_lanes.clear();
  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(2, 1, 1),
    ignition::math::Vector3d(0., 20.0, 0.0)));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(2, 1, 2),
    ignition::math::Vector3d(10.0, 20.0, 0.0),
    false, true));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(2, 1, 3),
    ignition::math::Vector3d(40.0, 20.0, 0.0)));
  Connection l(std::to_string(2),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }
  {
  std::vector<DirectedWaypoint> waypoints;
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(2, 2, 1),
    ignition::math::Vector3d(10., 30.0, 0.0),
    true, false));
  waypoints.push_back(DirectedWaypoint(
    ignition::rndf::UniqueId(2, 2, 2),
    ignition::math::Vector3d(0.0, 30.0, 0.0)));
  Connection l(std::to_string(2),
    waypoints,
    width);
  connected_lanes.push_back(l);
  }
  builder->CreateSegmentConnections(2, &connected_lanes);

  builder->CreateConnection(width, ignition::rndf::UniqueId(1, 1, 2),
    ignition::rndf::UniqueId(2, 2, 1));
  builder->CreateConnection(width, ignition::rndf::UniqueId(2, 1, 2),
    ignition::rndf::UniqueId(1, 2, 2));
  builder->CreateConnection(width, ignition::rndf::UniqueId(2, 1, 2),
    ignition::rndf::UniqueId(1, 1, 3));

  auto road_geometry = builder->Build({"MultilaneLaneCross"});
  EXPECT_NE(road_geometry, nullptr);

  EXPECT_EQ(road_geometry->num_junctions(), 11);

  // Here I check for the lane creation, naming, and bound coordinates
  int junction_id;
  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-0-0"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1-0-0"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_1_1-1_1_2"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(20., 50.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-0-1"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1-0-1"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_1_2-1_1_3"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-0-2"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1-0-2"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_1_3-1_1_4"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(20.0, 0.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-1-0"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1-1-0"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_2_1-1_2_2"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(30.0, 0.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-1-1"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1-1-1"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_2_2-1_2_3"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(30.0, 50.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:2-0-0"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:2-0-0"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:2_1_1-2_1_2"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(0.0, 20.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:2-0-1"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:2-0-1"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:2_1_2-2_1_3"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(40.0, 20.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:2-1-0"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:2-1-0"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:2_2_1-2_2_2"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(10.0, 30.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(0.0, 30.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1_1_2-2_2_1"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:1_1_2-2_2_1"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:1_1_2-2_2_1"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(20.0, 40.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(10.0, 30.0, 0.0), kLinearTolerance);
  }
  {
  junction_id = FindJunction(*road_geometry, std::string("j:2_1_2-1_2_2"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:2_1_2-1_2_2"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:2_1_2-1_2_2"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(30.0, 30.0, 0.0), kLinearTolerance);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:2_1_2-1_1_3"));
  EXPECT_NE(junction_id, -1);
  EXPECT_EQ(road_geometry->junction(junction_id)->num_segments(), 1);
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->id().id,
    std::string("s:2_1_2-1_1_3"));
  EXPECT_EQ(road_geometry->junction(junction_id)->segment(0)->num_lanes(), 1);

  auto lane = road_geometry->junction(junction_id)->segment(0)->lane(0);
  EXPECT_EQ(lane->id().id, std::string("l:2_1_2-1_1_3"));
  EXPECT_GEO_NEAR(lane->ToGeoPosition(api::LanePosition(0.0, 0.0, 0.0)),
    api::GeoPosition(10.0, 20.0, 0.0), kLinearTolerance);
  EXPECT_GEO_NEAR(lane->ToGeoPosition(
      api::LanePosition(lane->length(), 0.0, 0.0)),
    api::GeoPosition(20.0, 10.0, 0.0), kLinearTolerance);
  }

  EXPECT_EQ(road_geometry->num_branch_points(), 12);
  // Checks for the branch points
  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-0-0"));
  auto branch_point = road_geometry->junction(junction_id)->segment(0)->
    lane(0)->GetBranchPoint(api::LaneEnd::kFinish);
  EXPECT_NE(branch_point, nullptr);
  EXPECT_EQ(branch_point->GetASide()->size(), 1);
  EXPECT_EQ(branch_point->GetBSide()->size(), 2);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-0-1"));
  auto branch_point = road_geometry->junction(junction_id)->segment(0)->
    lane(0)->GetBranchPoint(api::LaneEnd::kFinish);
  EXPECT_NE(branch_point, nullptr);
  EXPECT_EQ(branch_point->GetASide()->size(), 2);
  EXPECT_EQ(branch_point->GetBSide()->size(), 1);
  }

  {
  junction_id = FindJunction(*road_geometry, std::string("j:1-1-1"));
  auto branch_point = road_geometry->junction(junction_id)->segment(0)->
    lane(0)->GetBranchPoint(api::LaneEnd::kStart);
  EXPECT_NE(branch_point, nullptr);
  EXPECT_EQ(branch_point->GetASide()->size(), 2);
  EXPECT_EQ(branch_point->GetBSide()->size(), 1);
  }
}

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
