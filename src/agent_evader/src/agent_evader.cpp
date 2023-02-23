#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdlib.h>

#include <iostream>

#include "Dubins.hpp"
#include "DubinsArc.hpp"
#include "DubinsStructure.hpp"

#include "RoutePlanner.hpp"

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

class EvaderNode : public rclcpp::Node {

public:
	EvaderNode() : Node("agent_evader"), count_(0)  {

		auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

		dubins_ = new Dubins();

		sub_pers_pose = this->create_subscription<geometry_msgs::msg::TransformStamped>(
				"shelfino1/transform", qos, std::bind(&EvaderNode::retrieve_persecutor_position_tf, this, _1));
		sub_evader_pose = this->create_subscription<geometry_msgs::msg::TransformStamped>(
				"shelfino2/transform", qos, std::bind(&EvaderNode::retrieve_evader_position_tf, this, _1));
		sub_roadmap = this->create_subscription<graph_msgs::msg::GeometryGraph>(
				"roadmap", qos, std::bind(&EvaderNode::retrieve_roadmap, this, _1));
		sub_gates = this->create_subscription<geometry_msgs::msg::PoseArray>(
				"gate_position", qos, std::bind(&EvaderNode::retrieve_gates, this, _1));

		goal_options_ = rclcpp_action::Client<FollowPath>::SendGoalOptions();
		goal_options_.result_callback = std::bind(&EvaderNode::check_goal, this, _1);
		client_ptr_ = rclcpp_action::create_client<FollowPath>(this,"shelfino2/follow_path");
	}

private:

	void retrieve_persecutor_position_tf(const geometry_msgs::msg::TransformStamped::SharedPtr t) {

		RCLCPP_DEBUG(this->get_logger(), "[EvaderNode] Pursuer found in (%f, %f) th=%f", t->transform.translation.x, t->transform.translation.y, t->transform.rotation.w);

		persecutorPosition.transform.translation.x = t->transform.translation.x;
		persecutorPosition.transform.translation.y = t->transform.translation.y;
		persecutorPosition.transform.rotation.w = t->transform.rotation.w;
	}

	void retrieve_evader_position_tf(const geometry_msgs::msg::TransformStamped::SharedPtr t) {
		RCLCPP_INFO(this->get_logger(), "[EvaderNode] Evader found in (%f, %f) th=%f", t->transform.translation.x, t->transform.translation.y, t->transform.rotation.w);

		evaderPosition.transform.translation.x = t->transform.translation.x;
		evaderPosition.transform.translation.y = t->transform.translation.y;
		evaderPosition.transform.rotation.w = t->transform.rotation.w;
	}

	void retrieve_roadmap(const graph_msgs::msg::GeometryGraph::SharedPtr graph_msg){

		if(FLAG_ROADMAP_READY) return;

		RCLCPP_INFO(this->get_logger(), "[EvaderNode] Info with the roadmap received. It has %zu nodes", graph_msg->nodes.size());

		if(graph_msg->nodes.size()!=0){
			roadmap = graph_msg;
			FLAG_ROADMAP_READY = true;

			if(FLAG_GATE_READY && FLAG_ROADMAP_READY){
				start_motion();
			}
		}
	}

	void retrieve_gates(const geometry_msgs::msg::PoseArray::SharedPtr gates_msg){

			if(FLAG_GATE_READY) return;

			RCLCPP_INFO(this->get_logger(), "[EvaderNode] Info about the gates received. There are %zu gate(s)", gates_msg->poses.size());

			if(sizeof(gates_msg->poses.size())!=0){
				srand (time(NULL));
			  int gate_index = rand() % gates_msg->poses.size();

				targetGate = gates_msg->poses[gate_index];
				FLAG_GATE_READY = true;

				if(FLAG_GATE_READY && FLAG_ROADMAP_READY){
					start_motion();
				}
			}
	}

	uint32_t identify_gate(){

		GateCandidate candidate = GateCandidate();

		// We check all the points in the roadmap
		for (uint32_t nodeId = 0; nodeId < roadmap->nodes.size(); nodeId++){
			geometry_msgs::msg::Point node = roadmap->nodes[nodeId];

			double distance = std::sqrt(std::pow(node.x - targetGate.position.x, 2) + std::pow(node.y - targetGate.position.y, 2));
			if(candidate.distance > distance){
				candidate.id_in_roadmap = nodeId;
				candidate.distance = distance;
			}
		}

		RCLCPP_INFO(this->get_logger(), "[EvaderNode] The target gate is the node with id %u\n", candidate.id_in_roadmap);
		return candidate.id_in_roadmap;
	}


	void start_motion(){

		RCLCPP_INFO(this->get_logger(), "---> start_motion()");

		uint32_t gateIndex = identify_gate();
		geometry_msgs::msg::Point evader;
		std::list<geometry_msgs::msg::Point> milestones;

		nav_msgs::msg::Path path;
		path.header.stamp = this->get_clock()->now();
		path.header.frame_id = "map";

		evader.x = evaderPosition.transform.translation.x;
		evader.y = evaderPosition.transform.translation.y;

		route_planner_ = new RoutePlanner(roadmap, gateIndex);
		milestones = route_planner_->retrievePath(evader);

		RCLCPP_INFO(this->get_logger(), "Last milestone: (%f, %f)", milestones.back().x, milestones.back().y);

		dubins_->dubins_full_path(evaderPosition.transform.rotation.w, path.header.stamp, milestones, path.poses);
		RCLCPP_INFO(this->get_logger(), "A path has been generated with %lu steps", path.poses.size());
		send_path_command(path);

		pose_list = path.poses;
		RCLCPP_INFO(this->get_logger(), "<--- stop_motion()");
	}

	bool check_point_between(geometry_msgs::msg::Point pose_a, geometry_msgs::msg::Point pose_b){
		bool in_between_x = pose_a.x <= evaderPosition.transform.translation.x && evaderPosition.transform.translation.x <= pose_b.x;
		in_between_x = in_between_x || (pose_b.x <= evaderPosition.transform.translation.x && evaderPosition.transform.translation.x <=pose_a.x);

		bool in_between_y = pose_a.y <= evaderPosition.transform.translation.y && evaderPosition.transform.translation.y <= pose_b.y;
		in_between_y = in_between_y || (pose_b.y <= evaderPosition.transform.translation.y && evaderPosition.transform.translation.y <=pose_a.y);

		return in_between_x && in_between_y;
	}

	bool check_point(geometry_msgs::msg::Point pose){

		float distance_to_pose = std::sqrt(std::pow(pose.x - evaderPosition.transform.translation.x, 2) + std::pow(pose.y - evaderPosition.transform.translation.y, 2));
		if(distance_to_pose <=threshold_position){
			RCLCPP_INFO(this->get_logger(), "Pose: (%f, %f) and the Position (%f, %f)", pose.x, evaderPosition.transform.translation.x,pose.y, evaderPosition.transform.translation.y);
			RCLCPP_INFO(this->get_logger(), "Distance %f (%s)", distance_to_pose, distance_to_pose <=threshold_position?"true":"false");
		}

		return distance_to_pose <=threshold_position;
	}

	bool check_goal_reached(geometry_msgs::msg::Point pose){

		float distance_to_pose = std::sqrt(std::pow(pose.x - evaderPosition.transform.translation.x, 2) + std::pow(pose.y - evaderPosition.transform.translation.y, 2));
		return distance_to_pose <=threshold_goal;
	}

	bool check_interception(){

		float distance_to_pose = std::sqrt(std::pow(persecutorPosition.transform.translation.x - evaderPosition.transform.translation.x, 2) + std::pow(persecutorPosition.transform.translation.y - evaderPosition.transform.translation.y, 2));
		return distance_to_pose <=threshold_interception;
	}

	void check_goal(const GoalHandle::WrappedResult& result){

			if(result.code ==rclcpp_action::ResultCode::SUCCEEDED){
					RCLCPP_INFO(this->get_logger(), "Goal reached");
			}
			else if (result.code == rclcpp_action::ResultCode::ABORTED){
				if(check_interception()){
			 		RCLCPP_INFO(this->get_logger(), "Evader has been intercepted!!");
			 	}
				else if(check_goal_reached(pose_list.back().pose.position)){
						RCLCPP_INFO(this->get_logger(), "Evader is close enough to the gate, the path has been completed");
				}
				else{
					RCLCPP_INFO(this->get_logger(), "Path order expired, sending it again");
					time_t now = time(0);
					if(difftime(now, last_send)>=0.5){
						std::vector<geometry_msgs::msg::PoseStamped> copy_pose_list = pose_list;
						while(pose_list.size()>1){
							pose_list.erase(pose_list.begin());
							if(check_point(pose_list[0].pose.position))
								break;
						}
						if(pose_list.size()==1){
							pose_list = copy_pose_list;
						}
					}

					RCLCPP_INFO(this->get_logger(), "Resending the path with %lu steps", pose_list.size());

					nav_msgs::msg::Path path;
					path.header.stamp = this->get_clock()->now();
			    path.header.frame_id = "map";
					path.poses = pose_list;

					send_path_command(path);
				}
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
		last_send = time(0);

		RCLCPP_INFO(this->get_logger(), "<--- stop send_path_command()");
	}



	size_t count_;

	bool FLAG_ROADMAP_READY = false;
	bool FLAG_GATE_READY = false;

	time_t last_send;
	float threshold_position = 0.3;
	float threshold_goal = 0.6;
	float threshold_interception= 0.8;

	Dubins *dubins_;
	RoutePlanner *route_planner_;

	graph_msgs::msg::GeometryGraph::SharedPtr roadmap;

	TransformStamped persecutorPosition;
	TransformStamped evaderPosition;
	geometry_msgs::msg::Pose targetGate;

	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_pers_pose;
	rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr sub_evader_pose;
	rclcpp::Subscription<graph_msgs::msg::GeometryGraph>::SharedPtr sub_roadmap;
	rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_gates;

	rclcpp_action::Client<FollowPath>::SharedPtr client_ptr_;
	std::vector<geometry_msgs::msg::PoseStamped> pose_list;
	rclcpp_action::Client<FollowPath>::SendGoalOptions goal_options_;

};



int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<EvaderNode>());
	rclcpp::shutdown();
	return 0;
}
