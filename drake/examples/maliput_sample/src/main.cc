#include <iostream>
#include <string>
#include <memory>
#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/api/road_geometry.h"
#include "drake/automotive/maliput/api/segment.h"
#include "drake/automotive/maliput/api/lane.h"
#include "drake/automotive/maliput/api/junction.h"
#include "drake/automotive/maliput/api/branch_point.h"

int main(int argc, char **argv) {
	if (argc < 2) {
		std::cerr << "Error. You should call: bazel run drake/examples/maliput_sample:sample <monolane_sample.yaml>" << std::endl;
		exit(-1);
	}
	std::string fileName(argv[1]);
	std::cout << "Opening " << fileName << std::endl;

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
				std::cout << "----Lane: " << lane->id().id << "| Length: " << lane->length() << std::endl;
	  		}
		}
	}
	return 0;
}