#include "drake/automotive/maliput/oneway/lane.h"

#include <memory>

#include "drake/automotive/maliput/oneway/branch_point.h"
#include "drake/automotive/maliput/oneway/road_geometry.h"
#include "drake/automotive/maliput/oneway/segment.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace maliput {
namespace oneway {

Lane::Lane(const Segment* segment, const api::LaneId& id,
           double length, const api::RBounds& bounds)
    : segment_(segment),
      id_(id),
      length_(length),
      bounds_(bounds) {
  DRAKE_DEMAND(segment != nullptr);
  // TODO(liang.fok) Consider initializing this variable in the constructor's
  // initializer list so branch_point_ can be declared `const`.
  branch_point_ = std::make_unique<BranchPoint>(
      api::BranchPointId({id.id + "_Branch_Point"}), this,
      segment->junction()->road_geometry());
}

const api::Segment* Lane::do_segment() const {
  return segment_;
}

const api::BranchPoint* Lane::DoGetBranchPoint(
    const api::LaneEnd::Which which_end) const {
  return branch_point_.get();
}

const api::LaneEndSet* Lane::DoGetConfluentBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetConfluentBranches({this, which_end});
}

const api::LaneEndSet* Lane::DoGetOngoingBranches(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetOngoingBranches({this, which_end});
}

std::unique_ptr<api::LaneEnd> Lane::DoGetDefaultBranch(
    api::LaneEnd::Which which_end) const {
  return branch_point_->GetDefaultBranch({this, which_end});
}

api::RBounds Lane::do_lane_bounds(double) const {
  return bounds_;
}

api::RBounds Lane::do_driveable_bounds(double) const {
  return bounds_;
}

api::LanePosition Lane::DoEvalMotionDerivatives(
    const api::LanePosition& position,
    const api::IsoLaneVelocity& velocity) const {
  return api::LanePosition(velocity.sigma_v, velocity.rho_v, velocity.eta_v);
}

api::GeoPosition Lane::DoToGeoPosition(
    const api::LanePosition& lane_pos) const {
  return {lane_pos.s, lane_pos.r, lane_pos.h};
}

api::Rotation Lane::DoGetOrientation(
    const api::LanePosition& lane_pos) const {
  return api::Rotation(0, 0, 0);  // roll, pitch, yaw.
}

api::LanePosition Lane::DoToLanePosition(
    const api::GeoPosition& geo_pos,
    api::GeoPosition* nearest_point,
    double* distance) const {
  DRAKE_ABORT();  // TODO(liangfok) Implement me.
}

}  // namespace oneway
}  // namespace maliput
}  // namespace drake
