#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>

#include "Dubins.hpp"
#include "DubinsArc.hpp"
#include "DubinsStructure.hpp"

#include "Predictor.hpp"

#include "std_msgs/msg/string.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"
#include "tf2_msgs/msg/tf_message.h"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "graph_msgs/msg/geometry_graph.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using FollowPath = nav2_msgs::action::FollowPath;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

class GateCandidate{
	public:
		uint32_t id_in_roadmap;
		double distance;

		GateCandidate() {
			id_in_roadmap = 0;
			distance = 999;
		}
};

class AgentNode : public rclcpp::Node {

public:
	AgentNode() : Node("agentNode"), count_(0)  {

		auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

		dubins_ = new Dubins();

		sub_pers_pose = this->create_subscription<geometry_msgs::msg::TransformStamped>(
				"shelfino1/transform", qos, std::bind(&AgentNode::retrieve_persecutor_position_tf, this, _1));
		sub_evader_pose = this->create_subscription<geometry_msgs::msg::TransformStamped>(
				"shelfino2/transform", qos, std::bind(&AgentNode::retrieve_evader_position_tf, this, _1));
		sub_roadmap = this->create_subscription<graph_msgs::msg::GeometryGraph>(
				"roadmap", qos, std::bind(&AgentNode::retrieve_roadmap, this, _1));
		sub_gates = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"gate_position", qos, std::bind(&AgentNode::retrieve_gates, this, _1));

		//publisher_ = this->create_publisher<nav_msgs::msg::Path>("shelfino1/plan", 10);

		goal_options_ = rclcpp_action::Client<FollowPath>::SendGoalOptions();
		goal_options_.result_callback = std::bind(&AgentNode::check_goal, this, _1);
		client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"shelfino1/follow_path");
	}

private:

	void print_with_time(const char* msg) {
		//const char* msg = "Presecutor position retrieved";
		//print_with_time(msg);
	    time_t now = time(0);
	    struct tm tstruct = *localtime(&now);

	    char buf[20];
	    strftime(buf, sizeof(buf), "%X", &tstruct);

	    RCLCPP_INFO(this->get_logger(), "[AgentNode] %s at %s", msg, buf);
	}

	void retrieve_persecutor_position_tf(const geometry_msgs::msg::TransformStamped::SharedPtr t) {

		RCLCPP_INFO(this->get_logger(), "[AgentNode] Persecutor found in (%f, %f) th=%f", t->transform.translation.x, t->transform.translation.y, t->transform.rotation.w);

		persecutorPosition.transform.translation.x = t->transform.translation.x;
		persecutorPosition.transform.translation.y = t->transform.translation.y;
		persecutorPosition.transform.rotation.w = t->transform.rotation.w;
	}

	void retrieve_evader_position_tf(const geometry_msgs::msg::TransformStamped::SharedPtr t) {
		RCLCPP_DEBUG(this->get_logger(), "[AgentNode] Evader found in (%f, %f) th=%f", t->transform.translation.x, t->transform.translation.y, t->transform.rotation.w);

		geometry_msgs::msg::Point position;
		position.x = t->transform.translation.x;
		position.y = t->transform.translation.y;

		if(FLAG_PREDICTOR_ACTIVE){
			RCLCPP_DEBUG(this->get_logger(), "[AgentNode] Updating the position of evader");
			predictor_->updateCurrentPosition(position);
			if(!FLAG_EVADER_LOCALIZED){
				RCLCPP_INFO(this->get_logger(), "[AgentNode] Searching closest node the evader (first node)");
				predictor_->findEvaderFirstNode();
				FLAG_EVADER_LOCALIZED = true;

				one_time_timer_ = this->create_wall_timer(2s, std::bind(&AgentNode::timer_firstPrediction, this));
			}
		}
	}

	void retrieve_roadmap(const graph_msgs::msg::GeometryGraph::SharedPtr graph_msg){

		if(FLAG_ROADMAP_READY) return;

		RCLCPP_INFO(this->get_logger(), "[AgentNode] Info with the roadmap received. It has %zu nodes", graph_msg->nodes.size());

		if(graph_msg->nodes.size()!=0){
			roadmap = graph_msg;
			FLAG_ROADMAP_READY = true;

			if(FLAG_GATES_READY && FLAG_ROADMAP_READY){
				start_predictor();
			}
		}
	}

	void retrieve_gates(const geometry_msgs::msg::PoseArray::SharedPtr gates_msg){

			if(FLAG_GATES_READY) return;

			RCLCPP_INFO(this->get_logger(), "[AgentNode] Info about the gates received. There are %zu gate(s)", gates_msg->poses.size());

			if(sizeof(gates_msg->poses.size())!=0){
				gates = gates_msg;
				FLAG_GATES_READY = true;

				if(FLAG_GATES_READY && FLAG_ROADMAP_READY){
					start_predictor();
				}
			}
	}

	void timer_firstPrediction(){

		// Track the next node for the first time
		uint32_t expectedNode = predictor_->predictNextNode();
		predictor_->setExpectedNodeId(expectedNode);

		// Cancel this timer
		one_time_timer_->cancel();
		periodic_timer_ = this->create_wall_timer(1s, std::bind(&AgentNode::timer_updatePrediction, this));

		// Find the targetGate
		FLAG_TARGET_GATE_PREDICTED = predictor_->predictTargetGate();
		RCLCPP_INFO(this->get_logger(), "[AgentNode] The result from trying to find the targetGate during the first try was sucess=%s\n", FLAG_TARGET_GATE_PREDICTED ? "true" : "false");
		plan_interception();

	}

	void timer_updatePrediction(){
		RCLCPP_DEBUG(this->get_logger(), "[AgentNode] Realizing periodical check of the prediction");

		uint32_t currentTarget = predictor_->getExpectedNodeId();
	  uint32_t reviewedTarget = predictor_->reviewPrediction();
		if(!FLAG_TARGET_GATE_PREDICTED || currentTarget!=reviewedTarget){
			RCLCPP_WARN(this->get_logger(), "[AgentNode] It's necessary to review the path to follow!");
			if(currentTarget!=reviewedTarget){
				RCLCPP_WARN(this->get_logger(), "[AgentNode] because the expected node is different");
				RCLCPP_INFO(this->get_logger(), "[AgentNode] Before it was %u and now it is %u\n", currentTarget, reviewedTarget);
			}
			else{
				RCLCPP_WARN(this->get_logger(), "[AgentNode] because we still don't have the gate");
			}
			FLAG_TARGET_GATE_PREDICTED = predictor_->predictTargetGate();
			plan_interception();
		}
	}

	std::list<uint32_t> identify_gates(){

		map<int, GateCandidate> candidates;

		for (uint32_t i = 0; i < gates->poses.size(); i++){
			candidates.insert({ i, GateCandidate()});
	  }

		// We check all the points in the roadmap
		for (uint32_t nodeId = 0; nodeId < roadmap->nodes.size(); nodeId++){
			geometry_msgs::msg::Point node = roadmap->nodes[nodeId];

			// And check if they are close to a gate
			for (uint32_t i = 0; i < gates->poses.size(); i++){
				double distance = std::sqrt(std::pow(node.x - gates->poses[i].position.x, 2) + std::pow(node.y - gates->poses[i].position.y, 2));
				if(candidates[i].distance > distance){
					candidates[i].id_in_roadmap = nodeId;
					candidates[i].distance = distance;
				}
		  }
		}

		// We create the final list
		std::list<uint32_t> toReturn;
		map<int, GateCandidate>::iterator it = candidates.begin();
		for (; it != candidates.end(); it++) {
			toReturn.push_back((it->second).id_in_roadmap);
			RCLCPP_INFO(this->get_logger(), "[AgentNode] Adding the gate with id %u (%f)\n", (it->second).id_in_roadmap, (it->second).distance);
	  }

		return toReturn;
	}


	void start_predictor(){

		RCLCPP_INFO(this->get_logger(), "---> start_predictor()");

		std::list<uint32_t> gateIndeces = identify_gates();
		predictor_ = new Predictor(roadmap, gateIndeces);
		FLAG_PREDICTOR_ACTIVE = true;

		RCLCPP_INFO(this->get_logger(), "<--- stop_predictor()");
	}

	void plan_interception(){

		RCLCPP_DEBUG(this->get_logger(), "---> start plan_interception()");

		geometry_msgs::msg::Point persecutor;
		std::list<geometry_msgs::msg::Point> milestones;

		nav_msgs::msg::Path path;
		path.header.stamp = this->get_clock()->now();
    path.header.frame_id = "map";


		persecutor.x = persecutorPosition.transform.translation.x;
		persecutor.y = persecutorPosition.transform.translation.y;

		if(!FLAG_TARGET_GATE_PREDICTED){
			RCLCPP_INFO(this->get_logger(), "Evader's target gate has not been predicted. Approching the evader");
			milestones = predictor_->towardsEvaderPath(persecutor);
		}
		else{
			RCLCPP_INFO(this->get_logger(), "Evader's target gate has been predicted. Calculating best intersection");
			milestones = predictor_->interceptingPath(persecutor);
		}

		RCLCPP_INFO(this->get_logger(), "Last milestone: (%f, %f)", milestones.back().x, milestones.back().y);

		dubins_->dubins_full_path(persecutorPosition.transform.rotation.w, path.header.stamp, milestones, path.poses);
		RCLCPP_INFO(this->get_logger(), "A path has been generated with %lu steps", path.poses.size());
		send_path_command(path);

		pose_list = path.poses;
		RCLCPP_DEBUG(this->get_logger(), "<--- stop plan_interception()");

	}

	bool check_point_between(geometry_msgs::msg::Point pose_a, geometry_msgs::msg::Point pose_b){
		bool in_between_x = pose_a.x <= persecutorPosition.transform.translation.x && persecutorPosition.transform.translation.x <= pose_b.x;
		in_between_x = in_between_x || (pose_b.x <= persecutorPosition.transform.translation.x && persecutorPosition.transform.translation.x <=pose_a.x);

		bool in_between_y = pose_a.y <= persecutorPosition.transform.translation.y && persecutorPosition.transform.translation.y <= pose_b.y;
		in_between_y = in_between_y || (pose_b.y <= persecutorPosition.transform.translation.y && persecutorPosition.transform.translation.y <=pose_a.y);

		return in_between_x && in_between_y;
	}

	void check_goal(const GoalHandle::WrappedResult& result){

			if(result.code ==rclcpp_action::ResultCode::SUCCEEDED){
					RCLCPP_INFO(this->get_logger(), "Goal reached");
			}
			else if (result.code == rclcpp_action::ResultCode::ABORTED){
					RCLCPP_INFO(this->get_logger(), "Path order expired, sending it again");
					bool still_path_left = true;
					while(still_path_left && pose_list.size()>1){
						pose_list.erase(pose_list.begin());
						still_path_left = check_point_between(pose_list[0].pose.position, pose_list[1].pose.position);
					}

					RCLCPP_INFO(this->get_logger(), "Resending the path with %lu steps", pose_list.size());

					nav_msgs::msg::Path path;
					path.header.stamp = this->get_clock()->now();
			    path.header.frame_id = "map";
					path.poses = pose_list;

					send_path_command(path);
			}
			else if (result.code == rclcpp_action::ResultCode::CANCELED){
					RCLCPP_ERROR(this->get_logger(), "Goal canceled");
			}
			else{
					RCLCPP_ERROR(this->get_logger(), "Unknown result code");
			}
	}

	void send_path_command(nav_msgs::msg::Path path){
		RCLCPP_INFO(this->get_logger(), "---> start send_path_command()");

		auto path_msg = FollowPath::Goal();
		path_msg.controller_id = "FollowPath";
		path_msg.path = path;

		if (!client_ptr_->wait_for_action_server()) {
				RCLCPP_ERROR(this->get_logger(), "Action server not found");
				exit(2);
		}

		client_ptr_->async_send_goal(path_msg, goal_options_);

		RCLCPP_INFO(this->get_logger(), "<--- stop send_path_command()");
	}



	size_t count_;

	bool FLAG_ROADMAP_READY = false;
	bool FLAG_GATES_READY = false;
	bool FLAG_PREDICTOR_ACTIVE = false;
	bool FLAG_EVADER_LOCALIZED = false;
	bool FLAG_TARGET_GATE_PREDICTED = false;
	bool FLAG_SENDING_PATH = false;

	Dubins *dubins_;
	Predictor *predictor_;

	graph_msgs::msg::GeometryGraph::SharedPtr roadmap;
	geometry_msgs::msg::PoseArray::SharedPtr gates;

	TransformStamped persecutorPosition;

	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_pers_pose;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_evader_pose;
	rclcpp::Subscription<graph_msgs::msg::GeometryGraph>::SharedPtr sub_roadmap;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates;

	rclcpp::TimerBase::SharedPtr one_time_timer_;
	rclcpp::TimerBase::SharedPtr periodic_timer_;
	rclcpp::TimerBase::SharedPtr search_gate_timer_;

	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
	rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
	std::vector<geometry_msgs::msg::PoseStamped> pose_list;
	rclcpp_action::Client<FollowPath>::SendGoalOptions goal_options_;

};



int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<AgentNode>());
	rclcpp::shutdown();
	return 0;
}
