#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

typedef enum direction_state
{
	Stop_State = 0,
	Move_Forward = 1,
	Move_Backward = 2,
	Rotate_Clockwise = 3,
	Rotate_CounterClockwise = 4

} direction_state;

typedef struct velocity
{
	double linear_velocity;
	double angular_velocity;
} velocity;

class SquareRoutine : public rclcpp::Node
{
	public:
		SquareRoutine() : Node("Square_Routine")
		{
			subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
			"odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
			timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this));
		}

	private:
		//Calculate yaw
		double calculate_yaw(double w, double x, double y, double z)
		{
			double siny_cosp = 2 * (w * z + x * y);
			double cosy_sinp = 1 - 2 * (y * y + z * z);
			return atan2(siny_cosp, cosy_sinp);
		}
		//change radians to degrees
		double rad_to_deg(double rad)
		{	
			return rad * (180 / M_PI);
		}
		//Normalize angle to -180 to 180 degrees
		double normalize_angle(double angle)
		{
			angle = fmod(angle + 180, 360);
			if (angle < 0)
			{
				angle += 360;
			}

			return angle - 180;
		}

		double add_angles(double angle_1, double angle_2)
		{
			return normalize_angle(angle_1 + angle_2);

		}
		
		//Calculate the distance based on X & Y
		double calculate_distance(double x_1, double x_2, double y_1, double y_2)
		{
			return pow(pow(x_2 - x_1, 2) + pow(y_2 - y_1, 2), 0.5);
		}
		void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
		{
			x_now = msg->pose.pose.position.x;
			y_now = msg->pose.pose.position.y;
			double qx = msg->pose.pose.orientation.x;
			double qy = msg->pose.pose.orientation.y;
			double qz = msg->pose.pose.orientation.z;
			double qw = msg->pose.pose.orientation.w;
			current_angle = rad_to_deg(calculate_yaw(qw, qx, qy, qz));
		}
		void timer_callback()
		{

		
			geometry_msgs::msg::Twist msg;
			velocity velocity;
			//calculate the distance from initial
			d_position = calculate_distance(x_now, x_init, y_now, y_init);
			//Find the difference of angles
			d_angle = add_angles(angle_aim, -current_angle);
			//Check the conditions
			if (d_position > d_position_aim)
			{
				last_state_complete = 1;
				d_position_aim = 100;
			}
			if (abs(d_angle) < angle_tolerance)
			{
				angle_tolerance = -1;
				last_state_complete = 1;
			}

			sequence_statemachine();
			velocity = direction_state_machine();
	
			msg.linear.x = velocity.linear_velocity;
			msg.angular.z = velocity.angular_velocity;
			publisher_->publish(msg);


		//RCLCPP_INFO(this->get_logger(), "Published cmd_vel.");
		}
		// Commands for direction
		velocity direction_state_machine()
		{
			velocity velocity = { 0, 0 };
			switch(direction)
			{
			case Move_Forward:
				velocity.linear_velocity = linear_speed;
				break;
	
			case Move_Backward:
				velocity.linear_velocity = -linear_speed;
				break;
	
			case Rotate_Clockwise:
				velocity.angular_velocity = -angular_speed;
				break;

			case Rotate_CounterClockwise:
				velocity.angular_velocity = angular_speed;
				break;

			case Stop_State:
				break;
			}
			return velocity;
		}
		//Commands to move a square path 
		void sequence_statemachine()
		{
			if (last_state_complete == 1)
			{
				switch (count_)
				{
				case 0:
					direction = Move_Forward;
					move_distance(1);
					break;
				case 1:
					direction = Rotate_Clockwise;
					rotate_angle(90);
					break;
				case 2:
					direction = Move_Forward;
					move_distance(1);
					break;
				case 3:
					direction = Rotate_Clockwise;
					rotate_angle(90);
					break;
				case 4:
					direction = Move_Forward;
					move_distance(1);
					break;
				case 5:
					direction = Rotate_Clockwise;
					rotate_angle(90);
					break;
				case 6:
					direction = Move_Forward;
					move_distance(1.0);
					break;
				case 7:
					direction = Rotate_Clockwise;
					rotate_angle(90);
					break;
				default:
					RCLCPP_INFO(this->get_logger(), "square completed");
					direction = Stop_State;
					break;
				}
			}
		}	

	// Set the initial position as where robot is now and put new d_aim in place	
	void move_distance(double distance)
	{
		d_position_aim = distance;
		x_init = x_now;
		y_init = y_now;
		count_++;		// advance state counter
		last_state_complete = 0;
	}
	//Set Angle Aim
	void rotate_angle(double angle)
	{
		angle_tolerance = 2;
		angle_aim = add_angles(current_angle, angle);
		count_++;  //advance state counter-
		last_state_complete = 0;
	}


	// Declaration of subscription_ attribute
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

	// Declaration of publisher_ attribute      
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

	// Declaration of the timer_ attribute
	rclcpp::TimerBase::SharedPtr timer_;

	// Declaration of Class Variables
	double linear_speed = 0.1;
	double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
	double qx = 0, qy = 0, qz = 0, qw = 0;
	double d_position = 0, d_position_aim = 0;
	double current_angle = 0;
	double d_angle = 0, angle_aim = 0;
	double angular_speed = -0.1, angle_tolerance = 2;

	direction_state direction = Stop_State;

	size_t count_ = 0;
	int last_state_complete = 1;


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