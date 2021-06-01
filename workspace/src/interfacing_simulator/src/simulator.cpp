#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ricaip_interfaces/srv/assigned_task.hpp"
#include "ricaip_interfaces/action/task_go_to.hpp"
#include "ricaip_interfaces/action/task_go_home.hpp"
#include "ricaip_interfaces/action/task_transport.hpp"

using namespace std::chrono_literals;

class SimulatorNode: public rclcpp::Node {
	public:
		SimulatorNode(): Node("simulator"), count(0) {
			publishers_pose["robot_1"] = create_publisher<geometry_msgs::msg::PoseStamped>("/factory/robot_1/pose", 1);
			publishers_pose["robot_2"] = create_publisher<geometry_msgs::msg::PoseStamped>("/factory/robot_2/pose", 1);

			publishers_status["robot_1"] = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/factory/robot_1/status", 1);
			publishers_status["robot_2"] = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("/factory/robot_2/status", 1);

			publishTimerPose = create_wall_timer(100ms, std::bind(&SimulatorNode::callback_pose_publishing, this));
			publishTimerStatus = create_wall_timer(1000ms, std::bind(&SimulatorNode::callback_status_publishing, this));
		
			r1_assigned_task_srv = create_service<ricaip_interfaces::srv::AssignedTask>("/factory/robot_1/assigned_task", std::bind(&SimulatorNode::r1_handle_service, this, std::placeholders::_1, std::placeholders::_2));

			r1_goto_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskGoTo>(
					this,
					"/factory/robot_1/task_go_to",
					std::bind(&SimulatorNode::r1_goto_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r1_goto_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r1_goto_handle_accepted, this, std::placeholders::_1));

			r1_gohome_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskGoHome>(
					this,
					"/factory/robot_1/task_go_home",
					std::bind(&SimulatorNode::r1_gohome_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r1_gohome_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r1_gohome_handle_accepted, this, std::placeholders::_1));

			r1_transport_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskTransport>(
					this,
					"/factory/robot_1/task_transport",
					std::bind(&SimulatorNode::r1_transport_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r1_transport_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r1_transport_handle_accepted, this, std::placeholders::_1));

			r2_assigned_task_srv = create_service<ricaip_interfaces::srv::AssignedTask>("/factory/robot_2/assigned_task", std::bind(&SimulatorNode::r2_handle_service, this, std::placeholders::_1, std::placeholders::_2));

			r2_goto_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskGoTo>(
					this,
					"/factory/robot_2/task_go_to",
					std::bind(&SimulatorNode::r2_goto_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r2_goto_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r2_goto_handle_accepted, this, std::placeholders::_1));

			r2_gohome_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskGoHome>(
					this,
					"/factory/robot_2/task_go_home",
					std::bind(&SimulatorNode::r2_gohome_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r2_gohome_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r2_gohome_handle_accepted, this, std::placeholders::_1));

			r2_transport_action_server = rclcpp_action::create_server<ricaip_interfaces::action::TaskTransport>(
					this,
					"/factory/robot_2/task_transport",
					std::bind(&SimulatorNode::r2_transport_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
					std::bind(&SimulatorNode::r2_transport_handle_cancel, this, std::placeholders::_1),
					std::bind(&SimulatorNode::r2_transport_handle_accepted, this, std::placeholders::_1));
		}

	private:
		void callback_pose_publishing() {
			rclcpp::Clock clk;
			tf2::Quaternion quat;
			auto message = geometry_msgs::msg::PoseStamped();

			// robot_1 pose generation
			message.header.frame_id = "map";
			message.header.stamp = clk.now();
			message.pose.position.x = 2 * std::cos(count * M_PI/180);
			message.pose.position.y = 3 * std::sin(count * M_PI/180);
			quat.setRPY(0, 0, 2 * count * M_PI/180);
			message.pose.orientation = tf2::toMsg(quat);
			publishers_pose["robot_1"]->publish(message);
			
			// robot_2 pose generation
			message.header.stamp = clk.now();
			if ((count / 100) % 2 == 0) {
				message.pose.position.x = position;
				position += 0.05;
				quat.setRPY(0, 0, 0);
				message.pose.orientation = tf2::toMsg(quat);
			} else {
				message.pose.position.x = position;
				position -= 0.05;
				quat.setRPY(0, 0, 180 * M_PI/180);
				message.pose.orientation = tf2::toMsg(quat);
			}
			message.pose.position.y = 0;
			publishers_pose["robot_2"]->publish(message);

			count++;
		}

		void callback_status_publishing() {
			auto message = diagnostic_msgs::msg::DiagnosticStatus();
			auto values = diagnostic_msgs::msg::KeyValue();

			// robot_1 status generation
			message.level = message.OK;
			message.hardware_id = "robot_1";
			values.key = "battery";
			values.value = "ok";
			message.values.push_back(values);
			values.key = "busy";
			robot_1_busy = robot_1_task_id.empty() ? "no" : "yes";
			values.value = robot_1_busy;
			message.values.push_back(values);
			values.key = "assigned_task";
			values.value = robot_1_task_id;
			message.values.push_back(values);
			publishers_status["robot_1"]->publish(message);

			// robot_2 status generation
			message.level = message.WARN;
			message.hardware_id = "robot_2";
			message.values.pop_back();
			message.values.pop_back();
			message.values.pop_back();
			values.key = "battery";
			values.value = "low";
			message.values.push_back(values);
			values.key = "busy";
			robot_2_busy = robot_2_task_id.empty() ? "no" : "yes";
			values.value = robot_2_busy;
			message.values.push_back(values);
			values.key = "assigned_task";
			values.value = robot_2_task_id;
			message.values.push_back(values);
			publishers_status["robot_2"]->publish(message);
		}

		void r1_handle_service(const std::shared_ptr<ricaip_interfaces::srv::AssignedTask::Request>,
                  std::shared_ptr<ricaip_interfaces::srv::AssignedTask::Response> response) {
			
			response->id = robot_1_task_id;
		}

		rclcpp_action::GoalResponse r1_goto_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskGoTo::Goal> goal) {
			
			robot_1_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r1_goto_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>>) {
			robot_1_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r1_goto_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>> goal_handle) {
			
			std::thread{std::bind(&SimulatorNode::r1_goto_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r1_goto_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskGoTo::Result>();

			for (int i = 0; i < 60;i++) {
				//std::cout << "working on go to task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_1_task_id = "";
				goal_handle->succeed(result);
			}
		}

		rclcpp_action::GoalResponse r1_gohome_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskGoHome::Goal> goal) {

			robot_1_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r1_gohome_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>>) {

			robot_1_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r1_gohome_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>> goal_handle) {

			std::thread{std::bind(&SimulatorNode::r1_gohome_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r1_gohome_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskGoHome::Result>();

			for (int i = 0; i < 30;i++) {
				//std::cout << "working on go home task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_1_task_id = "";
				goal_handle->succeed(result);
			}
		}

		rclcpp_action::GoalResponse r1_transport_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskTransport::Goal> goal) {

			robot_1_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r1_transport_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>>) {
			
			robot_1_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r1_transport_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>> goal_handle) {
			
			std::thread{std::bind(&SimulatorNode::r1_transport_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r1_transport_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskTransport::Result>();

			for (int i = 0; i < 90;i++) {
				//std::cout << "working on transport task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_1_task_id = "";
				goal_handle->succeed(result);
			}
		}

		void r2_handle_service(const std::shared_ptr<ricaip_interfaces::srv::AssignedTask::Request> request,
					std::shared_ptr<ricaip_interfaces::srv::AssignedTask::Response> response) {
			(void)request;

			response->id = robot_2_task_id;
		}

		rclcpp_action::GoalResponse r2_goto_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskGoTo::Goal> goal) {
			
			robot_2_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r2_goto_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>>) {
			
			robot_2_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r2_goto_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>> goal_handle) {
			
			std::thread{std::bind(&SimulatorNode::r2_goto_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r2_goto_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoTo>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskGoTo::Result>();

			for (int i = 0; i < 60;i++) {
				//std::cout << "working on go to task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_2_task_id = "";
				goal_handle->succeed(result);
			}
		}

		rclcpp_action::GoalResponse r2_gohome_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskGoHome::Goal> goal) {
			
			robot_2_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r2_gohome_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>>) {
			
			robot_2_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r2_gohome_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>> goal_handle) {
			
			std::thread{std::bind(&SimulatorNode::r2_gohome_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r2_gohome_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskGoHome>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskGoHome::Result>();

			for (int i = 0; i < 30;i++) {
				//std::cout << "working on go home task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_2_task_id = "";
				goal_handle->succeed(result);
			}
		}

		rclcpp_action::GoalResponse r2_transport_handle_goal (const rclcpp_action::GoalUUID &,
													std::shared_ptr<const ricaip_interfaces::action::TaskTransport::Goal> goal) {
			
			robot_2_task_id = goal->task_id;

			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		}

		rclcpp_action::CancelResponse r2_transport_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>>) {
			
			robot_2_task_id = "";
			return rclcpp_action::CancelResponse::ACCEPT;
		}

		void r2_transport_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>> goal_handle) {
			
			std::thread{std::bind(&SimulatorNode::r2_transport_execute, this, std::placeholders::_1), goal_handle}.detach();
		}

		void r2_transport_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ricaip_interfaces::action::TaskTransport>> goal_handle) {
			rclcpp::Rate loop_rate(1);
			auto result = std::make_shared<ricaip_interfaces::action::TaskTransport::Result>();

			for (int i = 0; i < 90;i++) {
				//std::cout << "working on transport task... " << i << std::endl;
				loop_rate.sleep();
			}

			if (rclcpp::ok()) {
				result->success = true;
				robot_2_task_id = "";
				goal_handle->succeed(result);
			}
		}

		rclcpp::TimerBase::SharedPtr publishTimerPose;
		rclcpp::TimerBase::SharedPtr publishTimerStatus;
		std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_pose;
		std::map<std::string, rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr> publishers_status;
		size_t count;
		float position = 0;
		std::string robot_1_task_id;
		std::string robot_2_task_id;
		std::string robot_1_busy;
		std::string robot_2_busy;

		rclcpp::Service<ricaip_interfaces::srv::AssignedTask>::SharedPtr r1_assigned_task_srv;

		rclcpp_action::Server<ricaip_interfaces::action::TaskGoTo>::SharedPtr r1_goto_action_server;
		rclcpp_action::Server<ricaip_interfaces::action::TaskGoHome>::SharedPtr r1_gohome_action_server;
		rclcpp_action::Server<ricaip_interfaces::action::TaskTransport>::SharedPtr r1_transport_action_server;

		rclcpp::Service<ricaip_interfaces::srv::AssignedTask>::SharedPtr r2_assigned_task_srv;

		rclcpp_action::Server<ricaip_interfaces::action::TaskGoTo>::SharedPtr r2_goto_action_server;
		rclcpp_action::Server<ricaip_interfaces::action::TaskGoHome>::SharedPtr r2_gohome_action_server;
		rclcpp_action::Server<ricaip_interfaces::action::TaskTransport>::SharedPtr r2_transport_action_server;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatorNode>());
  rclcpp::shutdown();
  return 0;
}