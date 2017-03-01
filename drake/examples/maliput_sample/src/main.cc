#include <iostream>
#include <string>
#include <memory>
#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/branch_point.h"
#include "drake/automotive/maliput/api/lane_data.h"

void PrintGeoCoordinatesOfRoad(const drake::maliput::api::Lane* lane, const double step);

void PrintGeoCoordinatesOfRoad(const drake::maliput::api::Lane* lane, const double step) {
	drake::maliput::api::LanePosition lp(0.0, 0.0, 0.0);

	while (lp.s < lane->length()) {
		auto gp = lane->ToGeoPosition(lp);
		auto rot = lane->GetOrientation(lp);
		std::cout << "------";
		std::cout << "[" << lp.s << ";" << lp.r << ";" << lp.h << "] --> ";
		std::cout << "[" << gp.x << ";" << gp.y << ";" << gp.z << "] | ";
		std::cout << "[" << rot.roll << ";" << rot.pitch << ";" << rot.yaw << "]";
	    std::cout << std::endl;
		lp.s += step;
	}
	lp.s = lane->length();
	auto gp = lane->ToGeoPosition(lp);
	auto rot = lane->GetOrientation(lp);
	std::cout << "------";
	std::cout << "[" << lp.s << ";" << lp.r << ";" << lp.h << "] --> ";
	std::cout << "[" << gp.x << ";" << gp.y << ";" << gp.z << "] | ";
	std::cout << "[" << rot.roll << ";" << rot.pitch << ";" << rot.yaw << "]";
	std::cout << std::endl;
}

int main(int argc, char **argv) {
	if (argc < 2) {
		std::cerr << "Error. You should call: bazel run drake/examples/maliput_sample:sample <monolane_sample.yaml>" << std::endl;
		exit(-1);
	}
	std::string fileName(argv[1]);
	std::cout << "Opening " << fileName << std::endl;

	const double step = 1.0;

	auto roadGeometry =
	  drake::maliput::monolane::LoadFile(fileName);
	// Walk the network.
	for (int ji = 0; ji < roadGeometry->num_junctions(); ++ji) {
		// Get the junction by index
		const drake::maliput::api::Junction* junction = roadGeometry->junction(ji);
		std::cout << "Junction id: " << junction->id().id << std::endl;
		// Iterate over the segments of the junction
		for (int si = 0; si < junction->num_segments(); ++si) {
			// Get the segment by index
			const drake::maliput::api::Segment* segment = junction->segment(si);
			std::cout << "--Segment: " << segment->id().id << " | # lanes: " << segment->num_lanes() << std::endl;
			// Iterate over the lanes
			for (int li = 0; li < segment->num_lanes(); ++li) {
				const drake::maliput::api::Lane* lane = segment->lane(li);
				std::cout << "----Lane: " << lane->id().id <<
					" | Length: " << lane->length() <<
					" | Lane bounds: [" << lane->lane_bounds(0.0).r_min << ";" << lane->lane_bounds(0.0).r_max << "]" <<
					" | Drivable bounds: [" << lane->driveable_bounds(0.0).r_min << ";" << lane->driveable_bounds(0.0).r_max << "]" <<
					std::endl;
				PrintGeoCoordinatesOfRoad(lane, step);
			}
		}
	}
	for (int bi = 0; bi < roadGeometry->num_branch_points(); ++bi) {
		const drake::maliput::api::BranchPoint *branchPoint = roadGeometry->branch_point(bi);
		std::cout << "Branch point: " << branchPoint->id().id << std::endl;
		// Get the lane end set A
		const drake::maliput::api::LaneEndSet *laneEndSetA =
			branchPoint->GetASide();
		for (int lei = 0; lei < laneEndSetA->size(); ++lei) {
			const drake::maliput::api::LaneEnd &laneEnd = laneEndSetA->get(lei);
			std::cout << "-- LaneEnd: [" << laneEnd.lane->id().id << ";";
			if (laneEnd.end == drake::maliput::api::LaneEnd::Which::kStart)
				std::cout << "Start]" << std::endl;
			else
				std::cout << "End]" << std::endl;
		}
	}
	return 0;
}