#include "graph_msgs/msg/geometry_graph.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "GatePath.hpp"

#include <cmath>
#include <cstdint>
#include <climits>
#include <utility>

#include <map>
#include <list>
#include <vector>

using namespace geometry_msgs::msg;

class RoutePlanner{

public:

	RoutePlanner();

	RoutePlanner(graph_msgs::msg::GeometryGraph::SharedPtr globalRoadmap, uint32_t gateIndex);

	std::list<geometry_msgs::msg::Point> retrievePath(geometry_msgs::msg::Point evader);

private:

	const int kNeighbours = 3;

	uint32_t RN;
	uint32_t targetGate;
	GatePath gateOptimalPath;
	graph_msgs::msg::GeometryGraph::SharedPtr roadmap;

	uint32_t findClosestNode(geometry_msgs::msg::Point position);
	double distanceToNode(geometry_msgs::msg::Point initPost, geometry_msgs::msg::Point targetPost);
	uint32_t shortestUnknownPath(double distFromInit[], bool checkedNodes[]);
	GatePath findOptimalPath(uint32_t initialNode);

};
