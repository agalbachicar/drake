#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "drake/automotive/maliput/rndf/junction.h"
#include "drake/automotive/maliput/rndf/lane.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/maliput/rndf/segment.h"
#include "drake/automotive/maliput/rndf/spline_lane.h"

namespace drake {
namespace maliput {
namespace rndf {

const double kLinearTolerance = 1e-3;
const double kAngularTolerance = 1e-3;

GTEST_TEST(RNDFSplineLanesTest, MetadataLane) {
  RoadGeometry rg({"FlatLineLane"}, kLinearTolerance, kAngularTolerance);
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  Lane *l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    {-5., 5.},
    {-10., 10.});

  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());

  // Check meta properties
  {
    EXPECT_EQ(l1->id().id, "l1");
    EXPECT_EQ(l1->segment(), s1);
    EXPECT_EQ(l1->index(), 0);
    EXPECT_EQ(l1->to_left(), nullptr);
    EXPECT_EQ(l1->to_right(), nullptr);
  }
}

GTEST_TEST(RNDFSplineLanesTest, BranchpointsLane) {
  RoadGeometry rg({"BranchpointsLane"}, kLinearTolerance, kAngularTolerance);
  Lane *l1, *l2;
  {
  Segment* s1 = rg.NewJunction({"j1"})->NewSegment({"s1"});
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(0.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  l1 = s1->NewSplineLane(
    {"l1"},
    control_points,
    {-5., 5.},
    {-10., 10.});
  }
  {
  Segment* s2 = rg.NewJunction({"j2"})->NewSegment({"s2"});
  std::vector<
    std::tuple<ignition::math::Vector3d,ignition::math::Vector3d>>
      control_points;
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(20.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  control_points.push_back(
    std::make_tuple(
      ignition::math::Vector3d(40.0, 0.0, 0.0),
      ignition::math::Vector3d(10.0, 0.0, 0.0)));
  l2 = s2->NewSplineLane(
    {"l2"},
    control_points,
    {-5., 5.},
    {-10., 10.});
  }
  BranchPoint *bp1, *bp2, *bp3;

  bp1 = rg.NewBranchPoint({"bp:" + std::to_string(1)});
  bp2 = rg.NewBranchPoint({"bp:" + std::to_string(2)});
  bp3 = rg.NewBranchPoint({"bp:" + std::to_string(3)});


  l1->SetStartBp(bp1);
  l1->SetEndBp(bp2);
  l2->SetStartBp(bp2);
  l2->SetEndBp(bp3);

  bp1->AddABranch({l1, api::LaneEnd::kStart});
  bp2->AddABranch({l1, api::LaneEnd::kFinish});
  bp2->AddBBranch({l2, api::LaneEnd::kStart});
  bp3->AddABranch({l2, api::LaneEnd::kFinish});

  // Check road geometry invariants
  EXPECT_EQ(rg.CheckInvariants(), std::vector<std::string>());


  EXPECT_EQ(l1->start_bp(), bp1);
  EXPECT_EQ(l1->end_bp(), bp2);
  EXPECT_EQ(l2->start_bp(), bp2);
  EXPECT_EQ(l2->end_bp(), bp3);
}


} // rndf
} // maliput
} // drake