/*
Code for DT1
V. Sieben
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>		// Timer functions
#include <functional>		// Arithmetic, comparisons, and logical operations
#include <memory>		// Dynamic memory management
#include <string>		// String functions
#include <cmath>

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"

// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
public:
	// Constructor creates a node named Square_Routine. 
	SquareRoutine() : Node("Square_Routine")
	{

		// Create the subscription
		// The callback function executes whenever data is published to the 'topic' topic.
		subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));

		// Create the publisher
		// Publisher to a topic named "topic". The size of the queue is 10 messages.
		publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

		// Create the timer
		timer_ = this->create_wall_timer(25ms, std::bind(&SquareRoutine::timer_callback, this));
	}

private:
	//update current position and orientation based on odometry data
	void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
	{
		x_now = msg->pose.pose.position.x;
		y_now = msg->pose.pose.position.y;
		double qx = msg->pose.pose.orientation.x;
		double qy = msg->pose.pose.orientation.y;
		double qw = msg->pose.pose.orientation.w;
		double qz = msg->pose.pose.orientation.z;
		tf2::Quaternion q(qx, qy, qz, qw);
		tf2::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		theta_now = yaw;
		RCLCPP_INFO(this->get_logger(), "Current Yaw : %lf\n", yaw);
		//RCLCPP_INFO(this->get_logger(), "Odom Acquired.")<;
	}

	void timer_callback()
	{


		geometry_msgs::msg::Twist msg;

		if (straight_)
		{

			// Calculate distance travelled from initial
			d_now = pow(pow(x_now - x_init, 2) + pow(y_now - y_init, 2), 0.5);

			// Keep moving if not reached last distance target
			if (d_now < d_aim)
			{
				msg.linear.x = x_vel; //Set linear velocity
				msg.angular.z = 0; //No angular velocity
				publisher_->publish(msg); //Publish the velocity command
			}
			else
			{
				//Stop and prepare to turn
				msg.linear.x = 0;
				msg.linear.z = 0;
				publisher_->publish(msg);
				straight_ = false;
				turn_ = false;
				theta_init = theta_now;
			}
		}
		// If done step, stop
		else
		{
		
			//turn 90 degrees
			double angle_ = std::abs(theta_now - theta_init);
			RCLCPP_INFO(this->get_logger(), "angle used for robot calculations: %lf\n", angle_);
			
			if (angle_ < M_PI / 2)  
			{
				msg.linear.x = 0; //double(rand())/double(RAND_MAX); //fun
				msg.angular.z = 0.2; //2*double(rand())/double(RAND_MAX) - 1; //fun
				publisher_->publish(msg);
			}
			else
			{
				msg.linear.x = 0;
				msg.angular.z = 0;
				publisher_->publish(msg);
				straight_ = true; //Prepare to turn straight again
				turn_ = true; //Set flag to turn completion
			}
		}
		if (turn_)
		{
			sequence_statemachine();
		}
			
	}
	
	
	// Manage states to create sequence of movements for square path
	void sequence_statemachine()
	{
		if (turn_)
		{
			switch (count_)
			{
			case 0:
				move_distance(1);
				break;
			case 1:
				move_distance(1);
				break;
			case 2:
				move_distance(1);
				break;
			case 3:
				move_distance(1);
				break;
			default:
				RCLCPP_INFO(this->get_logger(), "square completed");
				rclcpp::shutdown();
				break;
			}
		}
	}

	// Set the initial position as where robot is now and put new d_aim in place	
	void move_distance(double distance)
	{
		d_aim = distance;
		x_init = x_now;
		y_init = y_now;
		count_++;		// advance state counter
		turn_ = 0;
	}



	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;

	// Declaration of Class Variables
	double x_vel = 0.2;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
	double d_now = 0, d_aim = 0;
	double theta_now = 0, theta_init = 0;
	size_t count_ = 0;
	bool straight_;
	bool turn_;

};



//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char* argv[])
{
	// Initialize ROS2
	rclcpp::init(argc, argv);

	// Start node and callbacks
	rclcpp::spin(std::make_shared<SquareRoutine>());

	// Stop node 
	rclcpp::shutdown();
	return 0;
}

