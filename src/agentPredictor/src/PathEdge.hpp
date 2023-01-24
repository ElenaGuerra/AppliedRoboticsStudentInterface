#pragma once

#include <cstdint>
#include <map>
#include <vector>

class PathEdge {
public:
	uint32_t node_id;
	double weight; // Distance required to reach this point

  PathEdge();
	PathEdge(uint32_t id, double edgeWeight);
};
