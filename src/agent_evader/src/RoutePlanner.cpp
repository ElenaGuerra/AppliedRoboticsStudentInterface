#include "RoutePlanner.hpp"
#include <iostream>

PathEdge::PathEdge(){
	//...
}

PathEdge::PathEdge(uint32_t id, double edgeWeight) {
		node_id = id;
		weight = edgeWeight;
}

GatePath::GatePath() {
	//...
}

GatePath::GatePath(uint32_t id) {
	std::vector<PathEdge> selfPath;
	selfPath.push_back(PathEdge(id, 0));

	gate_id = id;
	path.insert({ id,selfPath });
}

RoutePlanner::RoutePlanner(){}

RoutePlanner::RoutePlanner(graph_msgs::msg::GeometryGraph::SharedPtr globalRoadmap, uint32_t gateIndex) {
	roadmap = globalRoadmap;
	targetGate = gateIndex;
	RN = globalRoadmap->nodes.size();

	gateOptimalPath = findOptimalPath(gateIndex);
}


std::list<geometry_msgs::msg::Point> RoutePlanner::retrievePath(geometry_msgs::msg::Point evader) {

	std::list<geometry_msgs::msg::Point> evaderPath;
	uint32_t evaderNodeId = findClosestNode(evader);

	// gateOptimalPath[targetNode] starts in the gate and ends in the evader
	// So we flip while getting the points
	for (uint32_t i = 0; i < gateOptimalPath.path[evaderNodeId].size(); i++) {
		uint32_t nodeToAdd = gateOptimalPath.path[evaderNodeId][i].node_id;
		evaderPath.push_back(roadmap->nodes[nodeToAdd]);
		std::cout<<nodeToAdd<<"<-";
	}
	std::cout<<std::endl;
	//evaderPath.push_front(evader);

	return evaderPath;
}


// Private

uint32_t RoutePlanner::findClosestNode(geometry_msgs::msg::Point position) {
	uint32_t closestNode = 0;
	double distance = 999;

	for (uint32_t node = 0; node < RN; node++) {
		double tempDistance = distanceToNode(position, roadmap->nodes[node]);
		if (tempDistance < distance) {
			closestNode = node;
			distance = tempDistance;
		}
	}

	return closestNode;
}

double RoutePlanner::distanceToNode(geometry_msgs::msg::Point initPost, geometry_msgs::msg::Point targetPost) {
	return std::sqrt(std::pow(targetPost.x - initPost.x, 2) + std::pow(targetPost.y - initPost.y, 2));
}

uint32_t RoutePlanner::shortestUnknownPath(double distFromInit[], bool checkedNodes[]) {
	int min = 999;
	uint32_t selectedChild = 0;

	for (uint32_t index = 0; index < RN; index++) {
		if (checkedNodes[index] == false && distFromInit[index] <= min) {
			min = distFromInit[index];
			selectedChild = index;
		}
	}
	return selectedChild;
}

GatePath RoutePlanner::findOptimalPath(uint32_t initialNode) {

	GatePath gatePath = GatePath(initialNode);
	double* dist = new double[RN];
	bool* checkedNodes = new bool[RN];

	for (uint32_t i = 0; i < RN; i++) {
		dist[i] = (i == initialNode) ? 0 : 999;
		checkedNodes[i] = false;
	}

	// Find shortest path for all vertices
	for (uint32_t i = 0; i < RN - 1; i++) {
		uint32_t nextNode = shortestUnknownPath(dist, checkedNodes);
		checkedNodes[nextNode] = true;

		int numEdges = roadmap->edges[nextNode].node_ids.size();
		for (int edgeIndex = 0; edgeIndex < numEdges; edgeIndex++) {

			// The edgeIndex-th edge referes to the edgeIndex-th node_id:
			uint32_t edgeId = roadmap->edges[nextNode].node_ids[edgeIndex];
			float edgeLength = roadmap->edges[nextNode].weights[edgeIndex];

			double currentDistance = dist[edgeId];
			double distWithNeighbour = dist[nextNode] + edgeLength;

			if (!checkedNodes[edgeId] && dist[nextNode] != 999 && (distWithNeighbour < currentDistance)) {
				dist[edgeId] = distWithNeighbour;
				gatePath.path[edgeId] = gatePath.path[nextNode];
				gatePath.path[edgeId].insert(gatePath.path[edgeId].begin(), PathEdge(edgeId, edgeLength));
			}
		}
	}

	delete[]dist;
	delete[]checkedNodes;

	return gatePath;
}
