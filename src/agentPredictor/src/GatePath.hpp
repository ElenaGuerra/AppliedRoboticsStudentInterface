#include <cstdint>
#include <map>
#include <vector>
#include "PathEdge.hpp"

class GatePath {
public:
	uint32_t gate_id;
	// Index of map: starting node
    // Vector: path the starting node to the target node (normally a gate)
	std::map<uint32_t, std::vector<PathEdge>> path;

	GatePath();
	GatePath(uint32_t gate_id);
};
