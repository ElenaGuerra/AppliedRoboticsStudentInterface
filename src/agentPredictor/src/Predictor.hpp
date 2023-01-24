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

class Predictor{

public:

	Predictor();

	Predictor(graph_msgs::msg::GeometryGraph::SharedPtr globalRoadmap, std::list<uint32_t> gateIndeces);

	uint32_t getExpectedNodeId();

	void setExpectedNodeId(uint nodeId);

	uint32_t getExpectedGateId();

	void findEvaderFirstNode();

	void updateCurrentPosition(geometry_msgs::msg::Point position);

	bool predictTargetGate();

	bool updatePredictionTargetGate(uint32_t discardedGate);

	uint32_t predictNextNode();

	uint32_t predictNextNodeByWindow();

	uint32_t reviewPrediction();

	std::list<geometry_msgs::msg::Point> interceptingPath(geometry_msgs::msg::Point persecutor);

	std::list<geometry_msgs::msg::Point> towardsEvaderPath(geometry_msgs::msg::Point persecutor);

private:

	const int kNeighbours = 3;
	const uint32_t windowSize = 20;
	const double thresholdCN = 0.5;
	const uint32_t differencesNeed = 3;

	uint32_t RN;
	graph_msgs::msg::GeometryGraph::SharedPtr roadmap;
	std::list<uint32_t> gates;
	std::map<uint32_t, GatePath> gatesOptimalPath;

	std::list<geometry_msgs::msg::Point> lastPositions;

	bool FLAG_CURRENTLY_CLOSE = false; // Mark if we already detected that the evader is near the current node
	uint32_t COUNTER_CHANGE_DETECTED=0;

	PathEdge currentNode;
	PathEdge expectedNode;
	uint32_t predictedGate;
	std::vector<PathEdge> expectedPath;

	uint32_t findClosestNode(geometry_msgs::msg::Point position);
	double distanceToNode(geometry_msgs::msg::Point initPost, geometry_msgs::msg::Point targetPost);

	uint32_t shortestUnknownPath(double distFromInit[], bool checkedNodes[]);
	GatePath findOptimalPath(uint32_t initialNode);

	std::vector<PathEdge> getPendingPath();
};
