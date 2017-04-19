#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/automotive_simulator.h"
#include "drake/automotive/create_trajectory_params.h"
#include "drake/automotive/gen/maliput_railcar_params.h"
#include "drake/automotive/maliput/api/lane_data.h"
#include "drake/automotive/maliput/dragway/road_geometry.h"
#include "drake/automotive/maliput/rndf/road_geometry.h"
#include "drake/automotive/monolane_onramp_merge.h"
#include "drake/automotive/rndf_generator.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"

DEFINE_int32(num_simple_car, 0, "Number of SimpleCar vehicles. The cars are "
             "named \"0\", \"1\", \"2\", etc. If this option is provided, "
             "simple_car_names must not be provided.");
DEFINE_string(simple_car_names, "",
              "A comma-separated list that specifies the number of SimpleCar "
              "models to instantiate, their names, and the names of the LCM "
              "channels to which they subscribe (e.g., 'Russ,Jeremy,Liang' "
              "would spawn 3 cars subscribed to DRIVING_COMMAND_Russ, "
              "DRIVING_COMMAND_Jeremy, and DRIVING_COMMAND_Liang). If this "
              "option is provided, num_simple_car must not be provided.");
DEFINE_int32(num_trajectory_car, 0, "Number of TrajectoryCar vehicles. This "
             "option is currently only applied when the road network is a flat "
             "plane or a dragway.");
DEFINE_int32(num_idm_controlled_maliput_railcar, 0, "Number of IDM-controlled "
             "MaliputRailcar vehicles. This option is currently only applied "
             "when the road network is a dragway. These cars are added after "
             "the trajectory cars are added but before the fixed-speed "
             "railcars are added. They are initialized to be behind the "
             "fixed-speed railcars, if any.");
DEFINE_int32(num_maliput_railcar, 0, "Number of fixed-speed MaliputRailcar "
             "vehicles. This option is currently only applied when the road "
             "network is a dragway or merge. The speed is derived based on the "
             "road's base speed and speed delta. The railcars are added after "
             "the IDM-controlled railcars are added and are positioned in "
             "front of the IDM-controlled railcars.");
DEFINE_double(target_realtime_rate, 1.0,
              "Playback speed.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

DEFINE_int32(num_dragway_lanes, 0,
             "The number of lanes on the dragway. The number of lanes is by "
             "default zero to disable the dragway. A dragway road network is "
             "only enabled when the user specifies a number of lanes greater "
             "than zero. Only one road network can be enabled. Thus if this "
             "option is enabled, no other road network can be enabled.");
DEFINE_double(dragway_length, 100, "The length of the dragway.");
DEFINE_double(dragway_lane_width, 3.7, "The dragway lane width.");
DEFINE_double(dragway_shoulder_width, 3.0, "The dragway's shoulder width.");
DEFINE_double(dragway_base_speed, 4.0,
              "The speed of the vehicles on the right-most lane of the "
              "dragway.");
DEFINE_double(dragway_lane_speed_delta, 2,
              "The change in vehicle speed in the left-adjacent lane. For "
              "example, suppose the dragway has 3 lanes. Vehicles in the "
              "right-most lane will travel at dragway_base_speed m/s. "
              "Vehicles in the middle lane will travel at "
              "dragway_base_speed + dragway_lane_speed_delta m/s. Finally, "
              "vehicles in the left-most lane will travel at "
              "dragway_base_speed + 2 * dragway_lane_speed_delta m/s.");
DEFINE_double(dragway_vehicle_delay, 3,
              "The starting time delay between consecutive vehicles on a "
              "lane.");

DEFINE_bool(with_onramp, false, "Loads the onramp road network. Only one road "
            "network can be enabled. Thus, if this option is enabled, no other "
            "road network can be enabled.");
DEFINE_double(onramp_base_speed, 25, "The speed of the vehicles added to the "
              "onramp.");
DEFINE_bool(onramp_swap_start, false, "Whether to swap the starting lanes of "
    "the vehicles on the onramp.");

DEFINE_bool(with_stalled_cars, false, "Places a stalled vehicle at the end of "
            "each lane of a dragway. This option is only enabled when the "
            "road is a dragway.");

DEFINE_bool(with_rndf, false, "Loads the rndf road network. Only one road "
            "network can be enabled. Thus, if this option is enabled, no other "
            "road network can be enabled.");
DEFINE_double(rndf_base_speed, 10, "The speed of the vehicles added to the "
              "rndf.");
DEFINE_double(rndf_delay, 5.0, "The starting time delay.");
DEFINE_string(lane_names, "",
  "A comma-separated list (e.g. 'lane_1,lane_2,lane_3' that generates a path "
  "for the car to follow.");

DEFINE_bool(build_T, false, "Build RNDF T example. It overrides the RNDF file.");

namespace drake {

using maliput::api::Lane;

namespace automotive {
namespace {

// The distance between the coordinates of consecutive rows of railcars on a
// dragway. 5 m ensures a gap between consecutive rows of Prius vehicles. It was
// empirically chosen.
constexpr double kRailcarRowSpacing{5};

enum class RoadNetworkType {
  flat = 0,
  dragway = 1,
  onramp = 2,
  rndf = 3
};

/*
std::string MakeChannelName(const std::string& name) {
  const std::string default_prefix{"DRIVING_COMMAND"};
  if (name.empty()) {
    return default_prefix;
  }
  return default_prefix + "_" + name;
}
*/


// Adds a MaliputRailcar to the simulation involving a dragway. It throws a
// std::runtime_error if there is insufficient lane length for adding the
// vehicle.
//
// @param num_cars The number of vehicles to add.
//
// @param idm_controlled Whether the vehicle should be IDM-controlled.
//
// @param initial_s_offset The initial s-offset against which all vehicles are
// added. The vehicles are added in each lane of the dragway starting at this
// s-offset. Each row of vehicles is in front of the previous row (increasing
// s).
//
// @param dragway_road_geometry The road on which to add the railcars.
//
// @param simulator The simulator to modify.
void AddMaliputRailcar(int num_cars, bool idm_controlled, int initial_s_offset,
    const maliput::dragway::RoadGeometry* dragway_road_geometry,
    AutomotiveSimulator<double>* simulator) {
  for (int i = 0; i < num_cars; ++i) {
    const int lane_index = i % FLAGS_num_dragway_lanes;
    const double speed = FLAGS_dragway_base_speed +
        lane_index * FLAGS_dragway_lane_speed_delta;
    const MaliputRailcarParams<double> params;
    const Lane* lane =
        dragway_road_geometry->junction(0)->segment(0)->lane(lane_index);
    MaliputRailcarState<double> state;
    const int row = i / FLAGS_num_dragway_lanes;
    const double s_offset = initial_s_offset + kRailcarRowSpacing * row;
    if (s_offset >= lane->length()) {
      throw std::runtime_error(
          "Ran out of lane length to add a MaliputRailcar.");
    }
    state.set_s(s_offset);
    state.set_speed(speed);
    if (idm_controlled) {
      simulator->AddIdmControlledPriusMaliputRailcar(
          "IdmControlledMaliputRailcar" + std::to_string(i),
          LaneDirection(lane), params, state);
    } else {
      simulator->AddPriusMaliputRailcar("MaliputRailcar" + std::to_string(i),
                                        LaneDirection(lane), params, state);
    }
  }
}

// Adds SimpleCar instances to the simulator. It uses FLAGS_num_simple_car or
// FLAGS_simple_car_names to determine the number and names of SimpleCar
// instances to add. If both are specified, an exception will be thrown. The
// SimpleCar instances will start at X = 0 in the world frame, and will be
// offset along the world frame's Y-axis by a constant distance.
void AddSimpleCars(AutomotiveSimulator<double>* simulator) {
  const double kSimpleCarYSpacing{3};
  if (FLAGS_num_simple_car != 0 && !FLAGS_simple_car_names.empty()) {
    throw std::runtime_error("Both --num_simple_car and --simple_car_names "
        "specified. Only one can be specified at a time.");
  }
  if (FLAGS_num_simple_car != 0 || !FLAGS_simple_car_names.empty()) {
    std::string simple_car_names = FLAGS_simple_car_names;
    if (FLAGS_simple_car_names.empty()) {
      for (int i = 0; i < FLAGS_num_simple_car; ++i) {
        if (i != 0) {
          simple_car_names += ",";
        }
        simple_car_names += std::to_string(i);
      }
    }
    std::istringstream simple_car_name_stream(simple_car_names);
    std::string name;
    double y_offset{0};
    while (getline(simple_car_name_stream, name, ',')) {
      const std::string& channel_name = MakeChannelName(name);
      drake::log()->info("Adding simple car subscribed to {}.", channel_name);
      SimpleCarState<double> state;
      state.set_y(y_offset);
      simulator->AddPriusSimpleCar(name, channel_name, state);
       y_offset += kSimpleCarYSpacing;
     }
  }
}

// Initializes the provided `simulator` with user-specified numbers of
// `SimpleCar` vehicles and `TrajectoryCar` vehicles. If parameter
// `road_network_type` equals `RoadNetworkType::dragway`, the provided
// `road_geometry` parameter must not be `nullptr`.
void AddVehicles(RoadNetworkType road_network_type,
    const maliput::api::RoadGeometry* road_geometry,
    AutomotiveSimulator<double>* simulator) {
  AddSimpleCars(simulator);
  if (road_network_type == RoadNetworkType::dragway) {
    DRAKE_DEMAND(road_geometry != nullptr);
    const maliput::dragway::RoadGeometry* dragway_road_geometry =
        dynamic_cast<const maliput::dragway::RoadGeometry*>(road_geometry);
    DRAKE_DEMAND(dragway_road_geometry != nullptr);
    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
      const int lane_index = i % FLAGS_num_dragway_lanes;
      const double speed = FLAGS_dragway_base_speed +
          lane_index * FLAGS_dragway_lane_speed_delta;
      const double start_time = i / FLAGS_num_dragway_lanes *
           FLAGS_dragway_vehicle_delay;
      const auto& params = CreateTrajectoryParamsForDragway(
          *dragway_road_geometry, lane_index, speed, start_time);
      simulator->AddPriusTrajectoryCar("TrajectoryCar" + std::to_string(i),
                                       std::get<0>(params),
                                       std::get<1>(params),
                                       std::get<2>(params));
    }
    AddMaliputRailcar(FLAGS_num_idm_controlled_maliput_railcar,
        true /* IDM controlled */, 0 /* initial s offset */,
        dragway_road_geometry, simulator);
    const double initial_s_offset =
      std::ceil(FLAGS_num_idm_controlled_maliput_railcar /
          FLAGS_num_dragway_lanes) * kRailcarRowSpacing;
    AddMaliputRailcar(FLAGS_num_maliput_railcar, false /* IDM controlled */,
        initial_s_offset, dragway_road_geometry, simulator);
    if (FLAGS_with_stalled_cars) {
      DRAKE_DEMAND(road_geometry != nullptr);
      for (int i = 0; i < FLAGS_num_dragway_lanes; ++i) {
        const Lane* lane = road_geometry->junction(0)->segment(0)->lane(i);
        DRAKE_DEMAND(lane != nullptr);
        const maliput::api::GeoPosition position = lane->ToGeoPosition(
            {lane->length() /* s */, 0 /* r */, 0 /* h */});
        SimpleCarState<double> state;
        state.set_x(position.x);
        state.set_y(position.y);
        simulator->AddPriusSimpleCar("StalledCar" + std::to_string(i),
            "StalledCarChannel" + std::to_string(i), state);
      }
    }
  } else if (road_network_type == RoadNetworkType::rndf) {
    DRAKE_DEMAND(road_geometry != nullptr);
    const maliput::rndf::RoadGeometry* rndf_road_geometry =
        dynamic_cast<const maliput::rndf::RoadGeometry*>(road_geometry);
    DRAKE_DEMAND(rndf_road_geometry != nullptr);

    std::vector<std::string> lane_name_paths;
    std::string testStr(FLAGS_lane_names);
    std::istringstream simple_lane_name_stream(FLAGS_lane_names);
    std::string lane_name;
    while (getline(simple_lane_name_stream, lane_name, ',')) {
      lane_name_paths.push_back(lane_name);
    }
    const auto& params = CreateTrajectoryParamsForRndf(
        *rndf_road_geometry, lane_name_paths, FLAGS_rndf_base_speed, FLAGS_rndf_delay);
    simulator->AddPriusTrajectoryCar("RNDFCar",
      std::get<0>(params),
      std::get<1>(params),
      std::get<2>(params));
  } else if (road_network_type == RoadNetworkType::onramp) {
    DRAKE_DEMAND(road_geometry != nullptr);
    for (int i = 0; i < FLAGS_num_maliput_railcar; ++i) {
      // Alternate starting the MaliputRailcar vehicles between the two possible
      // starting locations.
      const int n = FLAGS_onramp_swap_start ? (i + 1) : i;
      const std::string lane_name = (n % 2 == 0) ? "l:onramp0" : "l:pre0";
      const bool with_s = false;

      LaneDirection lane_direction(simulator->FindLane(lane_name), with_s);
      MaliputRailcarParams<double> params;
      params.set_r(0);
      params.set_h(0);
      MaliputRailcarState<double> state;
      state.set_s(with_s ? 0 : lane_direction.lane->length());
      state.set_speed(FLAGS_onramp_base_speed);
      simulator->AddPriusMaliputRailcar("MaliputRailcar" + std::to_string(i),
          lane_direction, params, state);
    }
  } else {
    for (int i = 0; i < FLAGS_num_trajectory_car; ++i) {
      const auto& params = CreateTrajectoryParams(i);
      simulator->AddPriusTrajectoryCar("TrajectoryCar" + std::to_string(i),
                                       std::get<0>(params),
                                       std::get<1>(params),
                                       std::get<2>(params));
    }
  }
}
/*
// Adds a flat terrain to the provided `simulator`.
void AddFlatTerrain(AutomotiveSimulator<double>* simulator) {
  // Intentially do nothing. This is possible since only non-physics-based
  // vehicles are supported and they will not fall through the "ground" when no
  // flat terrain is present.
  //
  // TODO(liang.fok): Once physics-based vehicles are supported, a flat terrain
  // should actually be added. This can be done by adding method
  // `AutomotiveSimulator::AddFlatTerrain()`, which can then call
  // `drake::multibody::AddFlatTerrainToWorld()`. This method is defined in
  // drake/multibody/rigid_body_tree_construction.h.
}
*/

/*
// Adds a dragway to the provided `simulator`. The number of lanes, lane width,
// lane length, and the shoulder width are all user-specifiable via command line
// flags.
const maliput::api::RoadGeometry* AddDragway(
    AutomotiveSimulator<double>* simulator) {
  std::unique_ptr<const maliput::api::RoadGeometry> road_geometry
      = std::make_unique<const maliput::dragway::RoadGeometry>(
          maliput::api::RoadGeometryId({"Automotive Demo Dragway"}),
          FLAGS_num_dragway_lanes,
          FLAGS_dragway_length,
          FLAGS_dragway_lane_width,
          FLAGS_dragway_shoulder_width);
  return simulator->SetRoadGeometry(std::move(road_geometry));
}
*/

// Adds a dragway to the provided `simulator`. The number of lanes, lane width,
// lane length, and the shoulder width are all user-specifiable via command line
// flags.
const maliput::api::RoadGeometry* AddRNDF(
    AutomotiveSimulator<double>* simulator) {
  auto onramp_generator = std::make_unique<RNDFTBuilder>();
  if (FLAGS_build_T) {
    return simulator->SetRoadGeometry(
      onramp_generator->Build());
  } else {
  const std::string road_waypoints =
"1.1.1\t9.999982 65.000912\n"
"1.1.2\t10.000045 65.000912\n"
"1.1.3\t10.000841 65.000912\n"
"1.1.4\t10.000967 65.000912\n"
"1.1.5\t10.001745 65.000912\n"
"1.1.6\t10.001871 65.000912\n"
"1.1.7\t10.002649 65.000912\n"
"1.1.8\t10.002776 65.000912\n"
"1.1.9\t10.003571 65.000912\n"
"1.1.10\t10.003634 65.000912\n"

"2.1.1\t9.999982 65.001824\n"
"2.1.2\t10.000045 65.001824\n"
"2.1.3\t10.000841 65.001824\n"
"2.1.4\t10.000967 65.001824\n"
"2.1.5\t10.001745 65.001824\n"
"2.1.6\t10.001871 65.001824\n"
"2.1.7\t10.002649 65.001824\n"
"2.1.8\t10.002776 65.001824\n"
"2.1.9\t10.003571 65.001824\n"
"2.1.10\t10.003634 65.001824\n"

"3.1.1\t9.999982 65.002736\n"
"3.1.2\t10.000045 65.002736\n"
"3.1.3\t10.000841 65.002736\n"
"3.1.4\t10.000967 65.002736\n"
"3.1.5\t10.001745 65.002736\n"
"3.1.6\t10.001871 65.002736\n"
"3.1.7\t10.002649 65.002736\n"
"3.1.8\t10.002776 65.002736\n"
"3.1.9\t10.003571 65.002736\n"
"3.1.10\t10.003634 65.002736\n"

"4.1.1\t10.000904 64.999982\n"
"4.1.2\t10.000904 65.000046\n"
"4.1.3\t10.000904 65.000848\n"
"4.1.4\t10.000904 65.000976\n"
"4.1.5\t10.000904 65.001760\n"
"4.1.6\t10.000904 65.001888\n"
"4.1.7\t10.000904 65.002672\n"
"4.1.8\t10.000904 65.002800\n"
"4.1.9\t10.000904 65.003603\n"
"4.1.10\t10.000904 65.003667\n"

"5.1.1\t10.001808 64.999982\n"
"5.1.2\t10.001808 65.000046\n"
"5.1.3\t10.001808 65.000848\n"
"5.1.4\t10.001808 65.000976\n"
"5.1.5\t10.001808 65.001760\n"
"5.1.6\t10.001808 65.001888\n"
"5.1.7\t10.001808 65.002672\n"
"5.1.8\t10.001808 65.002800\n"
"5.1.9\t10.001808 65.003603\n"
"5.1.10\t10.001808 65.003667\n"

"6.1.1\t10.002712 64.999982\n"
"6.1.2\t10.002712 65.000046\n"
"6.1.3\t10.002712 65.000848\n"
"6.1.4\t10.002712 65.000976\n"
"6.1.5\t10.002712 65.001760\n"
"6.1.6\t10.002712 65.001888\n"
"6.1.7\t10.002712 65.002672\n"
"6.1.8\t10.002712 65.002800\n"
"6.1.9\t10.002712 65.003603\n"
"6.1.10\t10.002712 65.003667\n"

"7.1.1\t9.999982 64.999982\n"
"7.1.2\t10.000027 65.000027\n"
"7.1.3\t10.000859 65.000867\n"
"7.1.4\t10.000949 65.000957\n"
"7.1.5\t10.001763 65.001779\n"
"7.1.6\t10.001853 65.001869\n"
"7.1.7\t10.002668 65.002691\n"
"7.1.8\t10.002757 65.002781\n"
"7.1.9\t10.003590 65.003621\n"
"7.1.10\t10.003634 65.003667\n"


"8.1.1\t10.003634 64.999982\n"
"8.1.2\t10.003590 65.000027\n"
"8.1.3\t10.002757 65.000867\n"
"8.1.4\t10.002668 65.000957\n"
"8.1.5\t10.001853 65.001779\n"
"8.1.6\t10.001763 65.001869\n"
"8.1.7\t10.000949 65.002691\n"
"8.1.8\t10.000859 65.002781\n"
"8.1.9\t10.000027 65.003621\n"
"8.1.10\t9.999982 65.003667\n"


"9.1.1\t9.999982 64.999982\n"
"9.1.2\t9.999982 65.000046\n"
"9.1.3\t9.999982 65.000848\n"
"9.1.4\t9.999982 65.001760\n"
"9.1.5\t9.999982 65.002672\n"
"9.1.6\t9.999982 65.003603\n"
"9.1.7\t9.999982 65.003667\n"


"10.1.1\t9.999982 65.003667\n"
"10.1.2\t10.000045 65.003667\n"
"10.1.3\t10.000967 65.003667\n"
"10.1.4\t10.001871 65.003667\n"
"10.1.5\t10.002776 65.003667\n"
"10.1.6\t10.003571 65.003667\n"
"10.1.7\t10.003634 65.003667\n"

"11.1.1\t10.003634 65.003667\n"
"11.1.2\t10.003634 65.003603\n"
"11.1.3\t10.003634 65.002672\n"
"11.1.4\t10.003634 65.001760\n"
"11.1.5\t10.003634 65.000848\n"
"11.1.6\t10.003634 65.000046\n"
"11.1.7\t10.003634 64.999982\n"

"12.1.1\t10.003634 64.999982\n"
"12.1.2\t10.003571 64.999982\n"
"12.1.3\t10.002776 64.999982\n"
"12.1.4\t10.001871 64.999982\n"
"12.1.5\t10.000967 64.999982\n"
"12.1.6\t10.000045 64.999982\n"
"12.1.7\t9.999982 64.999982\n";

  const std::string connections =
"1.1.3 4.1.4\n"
"1.1.5 5.1.4\n"
"1.1.7 6.1.4\n"

"2.1.3 4.1.6\n"
"2.1.5 5.1.6\n"

"2.1.7 6.1.6\n"
"3.1.3 4.1.8\n"

"3.1.5 5.1.8\n"
"3.1.7 6.1.8\n"
"4.1.3 1.1.4\n"
"4.1.5 2.1.4\n"
"4.1.7 3.1.4\n"

"5.1.3 1.1.6\n"
"5.1.5 2.1.6\n"

"5.1.7 3.1.6\n"
"6.1.3 1.1.8\n"

"6.1.5 2.1.8\n"
"6.1.7 3.1.8\n"

"2.1.5 7.1.6\n"
"1.1.3 7.1.4\n"
"6.1.7 7.1.8\n"
"7.1.3 1.1.4\n"
"7.1.3 4.1.4\n"
"7.1.5 2.1.6\n"
"7.1.5 5.1.6\n"
"7.1.7 3.1.8\n"
"7.1.7 6.1.8\n"
"3.1.7 7.1.8\n"
"4.1.3 7.1.4\n"
"5.1.5 7.1.6\n"

"4.1.7 8.1.8\n"
"5.1.5 8.1.6\n"
"6.1.3 8.1.4\n"
"8.1.3 1.1.8\n"
"8.1.3 6.1.4\n"
"8.1.5 2.1.6\n"
"8.1.5 5.1.6\n"
"8.1.5 7.1.6\n"
"8.1.7 3.1.4\n"
"8.1.7 4.1.8\n"
"7.1.5 8.1.6\n"
"3.1.3 8.1.8\n"
"2.1.5 8.1.6\n"
"1.1.7 8.1.4\n"

"9.1.3 1.1.2\n"
"9.1.4 2.1.2\n"
"9.1.5 3.1.2\n"
"9.1.6 10.1.2\n"
"12.1.6 9.1.2\n"

"8.1.9 10.1.2\n"
"5.1.9 10.1.4\n"
"4.1.9 10.1.3\n"
"6.1.9 10.1.5\n"
"10.1.6 11.1.2\n"

"7.1.9 11.1.2\n"
"1.1.9 11.1.5\n"
"2.1.9 11.1.4\n"
"3.1.9 11.1.3\n"
"11.1.6 12.1.2\n"
"11.1.6 8.1.2\n"

"12.1.3 6.1.2\n"
"12.1.4 5.1.2\n"
"12.1.5 4.1.2\n"
"12.1.6 7.1.2\n";

  return simulator->SetRoadGeometry(
    onramp_generator->Build(road_waypoints, connections));
  }
}

/*
// Adds a monolane-based onramp road network to the provided `simulator`.
const maliput::api::RoadGeometry* AddOnramp(
    AutomotiveSimulator<double>* simulator) {
  auto onramp_generator = std::make_unique<MonolaneOnrampMerge>();
  return simulator->SetRoadGeometry(onramp_generator->BuildOnramp());
}
*/

// Adds a terrain to the simulated world. The type of terrain added depends on
// the provided `road_network_type` parameter. A pointer to the road network is
// returned. A return value of `nullptr` is possible if no road network is
// added.
const maliput::api::RoadGeometry* AddTerrain(RoadNetworkType road_network_type,
    AutomotiveSimulator<double>* simulator) {
  const maliput::api::RoadGeometry* road_geometry{nullptr};

  /*switch (road_network_type) {
    case RoadNetworkType::flat: {
      AddFlatTerrain(simulator);
      break;
    }
    case RoadNetworkType::dragway: {
      road_geometry = AddDragway(simulator);
      break;
    }
    case RoadNetworkType::rndf: {
      road_geometry = AddRNDF(simulator);
      break;
    }
    case RoadNetworkType::onramp: {
      road_geometry = AddOnramp(simulator);
      break;
    }
  }
  return road_geometry;
  */
  road_geometry = AddRNDF(simulator);
  return road_geometry;
}

// Determines and returns the road network type based on the command line
// arguments.
RoadNetworkType DetermineRoadNetworkType() {
  /*
  int num_environments_selected{0};
  if (FLAGS_with_onramp) ++num_environments_selected;
  if (FLAGS_num_dragway_lanes) ++num_environments_selected;
  if (num_environments_selected > 1) {
    throw std::runtime_error("ERROR: More than one road network selected. Only "
        "one road network can be selected at a time.");
  }
  */

  return RoadNetworkType::rndf;
  /*
  if (FLAGS_num_dragway_lanes > 0) {
    return RoadNetworkType::dragway;
  } else if (FLAGS_with_onramp) {
    return RoadNetworkType::onramp;
  } else {
    return RoadNetworkType::flat;
  }
  */
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();
  const RoadNetworkType road_network_type = DetermineRoadNetworkType();
  auto simulator = std::make_unique<AutomotiveSimulator<double>>();
  const maliput::api::RoadGeometry* road_geometry =
      AddTerrain(road_network_type, simulator.get());
  AddVehicles(road_network_type, road_geometry, simulator.get());
  simulator->Start(FLAGS_target_realtime_rate);
  simulator->StepBy(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) { return drake::automotive::main(argc, argv); }
