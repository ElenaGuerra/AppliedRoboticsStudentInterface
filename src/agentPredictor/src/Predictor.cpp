#include "Predictor.hpp"

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

Predictor::Predictor(){}

Predictor::Predictor(graph_msgs::msg::GeometryGraph::SharedPtr globalRoadmap, std::list<uint32_t> gateIndeces) {
	roadmap = globalRoadmap;
	gates = gateIndeces;
	RN = sizeof(roadmap->nodes) / sizeof(geometry_msgs::msg::Point);

	for (uint32_t gateId : gates) {
		GatePath gatePath = findOptimalPath(gateId);
		gatesOptimalPath.insert({ gateId, gatePath });
	}
}

void Predictor::updateCurrentPosition(geometry_msgs::msg::Point position) {
	lastPositions.push_front(position);

	if (lastPositions.size() > windowSize) {
		lastPositions.pop_front();
	}
}

uint32_t Predictor::getExpectedNodeId(){
	return expectedNode.node_id;
}

void Predictor::setExpectedNodeId(uint nodeId){

	for (uint32_t i = 0; i < roadmap->edges.size(); i++) {
		if(nodeId==roadmap->edges[currentNode.node_id].node_ids[i]){
			expectedNode = PathEdge(nodeId,roadmap->edges[currentNode.node_id].weights[i]);
		}
	}

}

uint32_t Predictor::getExpectedGateId(){
	return predictedGate;
}

bool Predictor::predictTargetGate() {

	int validGates = 0;
	uint32_t tempGate = -1;
	std::vector<PathEdge> tempPath;

	uint32_t firstNode = currentNode.node_id;
	uint32_t secondNode = expectedNode.node_id;

	std::map<uint32_t, GatePath>::iterator it;

	for (it = gatesOptimalPath.begin(); it != gatesOptimalPath.end(); it++) {
		GatePath path2Gate = it->second;
		if (path2Gate.path[firstNode].size()>1 && path2Gate.path[firstNode][1].node_id == secondNode) {
			tempGate = path2Gate.gate_id;
			tempPath = path2Gate.path[firstNode];
			validGates++;
		}
	}

	if(validGates==1){
		predictedGate = tempGate;
		expectedPath = tempPath;
	}

	return validGates==1;
}

bool Predictor::updatePredictionTargetGate(uint32_t discardedGate) {

	int validGates = 0;
	uint32_t tempGate = -1;
	std::vector<PathEdge> tempPath;

	uint32_t firstNode = currentNode.node_id;
	uint32_t secondNode = expectedNode.node_id;

	std::map<uint32_t, GatePath>::iterator it;

	for (it = gatesOptimalPath.begin(); it != gatesOptimalPath.end(); it++) {
		GatePath path2Gate = it->second;
		if (it->first != discardedGate && path2Gate.path[firstNode].size()>1 && path2Gate.path[firstNode][1].node_id == secondNode) {
			//targetGate = it->first;
			tempGate = path2Gate.gate_id;
			tempPath = path2Gate.path[firstNode];
			validGates++;
			break;
		}
	}

	if(validGates==1){
		predictedGate = tempGate;
		expectedPath = tempPath;
	}

	return predictedGate;
}

uint32_t Predictor::predictNextNode() {
	uint32_t nextNode;
	double bestApproach = -1;

	for (uint32_t i = 0; i < roadmap->edges.size(); i++) {
		uint32_t edgeNodeId = roadmap->edges[currentNode.node_id].node_ids[i];
		geometry_msgs::msg::Point edgeNodePoint = roadmap->nodes[edgeNodeId];
		geometry_msgs::msg::Point currentNodePoint = roadmap->nodes[currentNode.node_id];

		double currentDistance = distanceToNode(lastPositions.front(), edgeNodePoint);
		double originalDistance = distanceToNode(currentNodePoint, edgeNodePoint);
		double temp = originalDistance == 0 ? 0 : currentDistance / originalDistance;
		if (bestApproach == -1 || temp < bestApproach) {
			bestApproach = temp;
			nextNode = edgeNodeId;
		}
	}

	return nextNode;
}

uint32_t Predictor::predictNextNodeByWindow() {
	uint32_t nextNode;
	double bestApproach = -1;

	for (uint32_t i = 0; i < roadmap->edges.size(); i++) {
		uint32_t edgeNodeId = roadmap->edges[currentNode.node_id].node_ids[i];
		geometry_msgs::msg::Point edgeNodePoint = roadmap->nodes[edgeNodeId];

		double currentDistance = distanceToNode(lastPositions.front(), edgeNodePoint);
		double previousDistance = distanceToNode(lastPositions.back(), edgeNodePoint);
		double temp = previousDistance == 0 ? 0 : currentDistance / previousDistance;
		if (bestApproach == -1 || temp < bestApproach) {
			bestApproach = temp;
			nextNode = edgeNodeId;
		}
	}
	return nextNode;
}

uint32_t Predictor::reviewPrediction() {

	uint32_t expectedTemp = expectedNode.node_id;
	geometry_msgs::msg::Point expectedPoint = roadmap->nodes[expectedNode.node_id];
	double distance2Target = distanceToNode(lastPositions.front(), expectedPoint);

	if (thresholdCN > distance2Target) {
		FLAG_CURRENTLY_CLOSE = true;
		geometry_msgs::msg::Point p = lastPositions.front();
		lastPositions.clear();
		lastPositions.push_back(p);
	}
	else if (FLAG_CURRENTLY_CLOSE) {
		FLAG_CURRENTLY_CLOSE = false;
		currentNode = expectedNode;
		for (uint32_t i = 0; i < expectedPath.size(); i++) {
			if (currentNode.node_id == expectedPath[i].node_id) {
				if (i == expectedPath.size() - 1) {
					//It remains the same, we're in the target gate
					expectedNode = expectedPath[i];
				}
				else {
					expectedNode = expectedPath[i + 1];
				}
				break;
			}
		}
	}
	else {
		// We need to detect the change a few times before accepting it
		expectedTemp = predictNextNode();
		if(expectedTemp!=expectedNode.node_id){
			COUNTER_CHANGE_DETECTED++;
			if(COUNTER_CHANGE_DETECTED>=differencesNeed){
				setExpectedNodeId(expectedTemp);
				COUNTER_CHANGE_DETECTED = 0;
			}
		}
		else{
			COUNTER_CHANGE_DETECTED = 0;
		}

	}
	return expectedNode.node_id;
}

std::list<geometry_msgs::msg::Point> Predictor::interceptingPath(geometry_msgs::msg::Point persecutor) {

	bool bestedAll = true;
	bool failedAll = true;
	std::vector<PathEdge> evaderPath = getPendingPath();
	std::list<geometry_msgs::msg::Point> persPath;

	uint32_t persNodeId = findClosestNode(persecutor);
	GatePath persRoadmap = findOptimalPath(persNodeId);
	geometry_msgs::msg::Point persNode = roadmap->nodes[persNodeId];

	uint32_t targetNode;
	uint32_t previousTargetNode = predictedGate;
	for (int numNodes = evaderPath.size(); numNodes > 0; numNodes--) {

		targetNode = evaderPath[numNodes-1].node_id;

		// Distance evader
		geometry_msgs::msg::Point firstNode = roadmap->nodes[evaderPath[0].node_id];
		double distEvader = distanceToNode(lastPositions.front(), firstNode);
		for (int i = 1; i < numNodes; i++) {
			distEvader += evaderPath[i].weight;
		}

		// Distance persecutor
		std::vector<PathEdge> tempPath = persRoadmap.path[targetNode];
		double distPersec = distanceToNode(persecutor, persNode);
		for (uint32_t i = 0; i < tempPath.size(); i++) {
			distPersec += tempPath[i].weight;
		}

		if (distPersec > distEvader) {
			bestedAll = false;
			break;
		}
		else {
			failedAll = false;
			previousTargetNode = targetNode;
		}
	}

	if (failedAll){
		// If pursuer didn't win any distance contest -> go to the gate
		targetNode = predictedGate;
	}
	else if (bestedAll){
		// The interception point is a middle node
		targetNode = previousTargetNode;
	}
	//If it bested all, targetNode already has the desired value

	// persRoadmap[targetNode] starts in the target and ends in the persecutor
	// So we flip while getting the points
	for (uint32_t i = 0; i < persRoadmap.path[targetNode].size(); i++) {
		uint32_t nodeToAdd = persRoadmap.path[targetNode][i].node_id;
		persPath.push_front(roadmap->nodes[nodeToAdd]);
	}
	persPath.push_front(persecutor);

	return persPath;

}

std::list<geometry_msgs::msg::Point> Predictor::towardsEvaderPath(geometry_msgs::msg::Point persecutor){

		std::list<geometry_msgs::msg::Point> persPath;
		uint32_t persNodeId = findClosestNode(persecutor);
		GatePath persRoadmap = findOptimalPath(persNodeId);

		uint32_t targetNode = expectedNode.node_id;

		for (uint32_t i = 0; i < persRoadmap.path[targetNode].size(); i++) {
			uint32_t nodeToAdd = persRoadmap.path[targetNode][i].node_id;
			persPath.push_front(roadmap->nodes[nodeToAdd]);
		}
		persPath.push_front(persecutor);

		return persPath;
}

void Predictor::findEvaderFirstNode(){
	if(lastPositions.size()!=0){
		uint32_t id = findClosestNode(lastPositions.back());
		currentNode = PathEdge(id,0);
	}
}

// Private

uint32_t Predictor::findClosestNode(geometry_msgs::msg::Point position) {
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

double Predictor::distanceToNode(geometry_msgs::msg::Point initPost, geometry_msgs::msg::Point targetPost) {
	return std::sqrt(std::pow(targetPost.x - initPost.x, 2) + std::pow(targetPost.y - initPost.y, 2));
}

uint32_t Predictor::shortestUnknownPath(double distFromInit[], bool checkedNodes[]) {
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

GatePath Predictor::findOptimalPath(uint32_t initialNode) {

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

		int numEdges = sizeof(roadmap->edges[nextNode].node_ids) / sizeof(uint32_t);
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

std::vector<PathEdge> Predictor::getPendingPath() {
	if (expectedPath[0].node_id == currentNode.node_id) {
		return expectedPath;
	}

	std::vector<PathEdge> leftPath;
	std::vector<PathEdge>::iterator it = expectedPath.end();
	for (; it != expectedPath.begin(); it--) {
		if ((*it).node_id == currentNode.node_id)
			break;
		else
			leftPath.insert(leftPath.begin(), (*it));
	}

	return leftPath;
}
