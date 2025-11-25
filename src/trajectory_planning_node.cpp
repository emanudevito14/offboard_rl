#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <string>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class TrajectoryPlanner : public rclcpp::Node
{
	public:
	TrajectoryPlanner() : Node("trajectory_planner")
	{
        for (const auto& name : wp_names)
        {
		 std::vector<double> wp;
		 this->declare_parameter("waypoints_list." + name, std::vector<double>{});
		 this->get_parameter("waypoints_list." + name, wp);

		 if (wp.size() != 4) {
			RCLCPP_ERROR(this->get_logger(),
							"Waypoint %s must contain 4 values [x,y,z,yaw]", name.c_str());
			continue;
		  }

          Eigen::Vector4d w(wp[0], wp[1], -wp[2], wp[3]);
          waypoints.push_back(w);
        }

		this->declare_parameter<double>("trajectory_params.segment_duration",1.0);
		this->get_parameter("trajectory_params.segment_duration", segment_duration_);

        RCLCPP_INFO(this->get_logger(), "Segment duration = %.2f", segment_duration_);
        
		std::cout << "waypoints received" << std::endl;
		std::cout << "segment_duration =" <<segment_duration_<< std::endl;
        
		traj_params_received=true;
        


		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

		local_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position",
		qos, std::bind(&TrajectoryPlanner::vehicle_local_position_callback, this, std::placeholders::_1));
		attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>("/fmu/out/vehicle_attitude",
		qos, std::bind(&TrajectoryPlanner::vehicle_attitude_callback, this, std::placeholders::_1));

		auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", cmd_qos);
		trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", cmd_qos);
		vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", cmd_qos);

		timer_offboard_ = this->create_wall_timer(100ms, std::bind(&TrajectoryPlanner::activate_offboard, this));
		timer_trajectory_publish_ = this->create_wall_timer(20ms, std::bind(&TrajectoryPlanner::publish_trajectory_setpoint, this));

		
	}

	private:
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_subscription_;
	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	rclcpp::TimerBase::SharedPtr timer_offboard_;
	rclcpp::TimerBase::SharedPtr timer_trajectory_publish_;

	std::thread keyboard_thread;

	std::vector<std::string> wp_names = {
    "wp_1", "wp_2", "wp_3", "wp_4", "wp_5", "wp_6", "wp_7"
    };
	std::vector<Eigen::Vector4d> waypoints;
	double segment_duration_;
    unsigned int number_path=0;
	bool traj_params_received{false};
	bool offboard_active{false};
	bool trajectory_computed{false};
	Eigen::Vector<double, 6> x; //coefficients of the trajectory polynomial

	double t{0.0};
	Eigen::Vector4d pos_init, pos_final;
	VehicleLocalPosition current_position_{};
	VehicleAttitude current_attitude_{};
	double offboard_counter{0};

	void vehicle_local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
	{
		current_position_ = *msg;
	}

	void vehicle_attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
	{
		current_attitude_ = *msg;
	}

	

	void activate_offboard()
	{
		if (traj_params_received)
		{
			if(offboard_counter == 10) {
				std::cout << "offboard_counter = 10" << std::endl;
				// Change to Offboard mode after 1 second of sending offboard messages
				VehicleCommand msg{};
				msg.param1 = 1;
				msg.param2 = 6;
				msg.command = VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
				msg.target_system = 1;
				msg.target_component = 1;
				msg.source_system = 1;
				msg.source_component = 1;
				msg.from_external = true;
				msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
				vehicle_command_publisher_->publish(msg);
                
				std::cout << "arm the vehicle" << std::endl;
				// Arm the vehicle
				msg.param1 = 1.0;
				msg.param2 = 0.0;
				msg.command = VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
				msg.target_system = 1;
				msg.target_component = 1;
				msg.source_system = 1;
				msg.source_component = 1;
				msg.from_external = true;
				msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
				vehicle_command_publisher_->publish(msg);

				// Set initial position
				pos_init(0) = current_position_.x;
				pos_init(1) = current_position_.y;
				pos_init(2) = current_position_.z;

				auto rpy = utilities::quatToRpy( Vector4d( current_attitude_.q[0], current_attitude_.q[1], current_attitude_.q[2], current_attitude_.q[3] ) );
				pos_init(3) = rpy[2];

				offboard_active = true;
			}

			OffboardControlMode msg{};
			msg.position = true;
			msg.velocity = false;
			msg.acceleration = false;
			msg.attitude = false;
			msg.body_rate = false;
			msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
			offboard_control_mode_publisher_->publish(msg);	

			if (offboard_counter < 11) offboard_counter++;
		}
	}

	void publish_trajectory_setpoint()
    {
    
        if (!traj_params_received || !offboard_active || number_path >= waypoints.size() - 1) {
        
          if (number_path >= waypoints.size() - 1) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory completed.");
           }
          return;
        }

		double dt = 1/50.0; // 20 ms
		TrajectorySetpoint msg{compute_trajectory_setpoint(t, number_path)};
		trajectory_setpoint_publisher_->publish(msg);
		t += dt;

    
        if (t > segment_duration_) {
        
			t = 0.0;
            number_path++;
            trajectory_computed = false;
      }
    }

	TrajectorySetpoint compute_trajectory_setpoint(double t_segment, unsigned int i)
    {   
		
		Vector4d initial_wp;
		Vector4d final_wp;

		if (i == 0) {
			initial_wp = pos_init;
			final_wp = pos_init+waypoints[0];
		} else {
			
			initial_wp = pos_init+waypoints[i-1];
			final_wp = pos_init+waypoints[i];
		}

		Vector4d e = final_wp - initial_wp;
		e(3) = utilities::angleError(final_wp(3), initial_wp(3));

		double s_f = e.head<3>().norm(); 
		if (s_f < 1e-3) {
			s_f = 1e-3; 
		}
		
		double V_intermediate = 1.0; 
		double A_intermediate = 0.0; 

		if (!trajectory_computed)
		{
			Eigen::VectorXd b(6);
			Eigen::Matrix<double, 6, 6> A;
		
			double T = segment_duration_;
			A << 0, 0, 0, 0, 0, 1,
				0, 0, 0, 0, 1, 0,
				0, 0, 0, 1, 0, 0,
				std::pow(T,5), std::pow(T,4), std::pow(T,3), std::pow(T,2), T, 1,
				5*std::pow(T,4), 4*std::pow(T,3), 3*std::pow(T,2), 2*T, 1, 0,
				20*std::pow(T,3), 12*std::pow(T,2), 6*T, 2, 0, 0;

			
			
			double V_start, A_start, V_end, A_end;

			if (i == 0) {
				
				V_start = 0.0; 
				A_start = 0.0;
			
				V_end = V_intermediate;
				A_end = A_intermediate;
				
			} else if (i < waypoints.size() - 1) {
			
				V_start = V_intermediate; 
				A_start = A_intermediate;
				V_end = V_intermediate;
				A_end = A_intermediate;
				
			} else {
				
				V_start = V_intermediate; 
				A_start = A_intermediate;
				
				V_end = 0.0;
				A_end = 0.0; 
			}

			b << 0.0, V_start, A_start, s_f, V_end, A_end;
			x = A.inverse() * b;
			trajectory_computed = true;
		}

		
		double s, s_d, s_dd;
		Eigen::Vector4d ref_traj_pos, ref_traj_vel, ref_traj_acc;
		double t_seg = t_segment; 

		s   = x(0) * std::pow(t_seg, 5.0)
			+ x(1) * std::pow(t_seg, 4.0)
			+ x(2) * std::pow(t_seg, 3.0)
			+ x(3) * std::pow(t_seg, 2.0)
			+ x(4) * t_seg
			+ x(5);

		
		s_d = 5.0  * x(0) * std::pow(t_seg, 4.0)
			+ 4.0  * x(1) * std::pow(t_seg, 3.0)
			+ 3.0  * x(2) * std::pow(t_seg, 2.0)
			+ 2.0  * x(3) * t_seg
			+        x(4);

		
		s_dd = 20.0 * x(0) * std::pow(t_seg, 3.0)
			+ 12.0 * x(1) * std::pow(t_seg, 2.0)
			+  6.0 * x(2) * t_seg
			+  2.0 * x(3); 

		
		ref_traj_pos.head<3>() = initial_wp.head<3>() + s * (e.head<3>() / s_f); 
		ref_traj_pos(3) = initial_wp(3) + s * (e(3) / s_f);
		ref_traj_vel.head<3>() = s_d * (e.head<3>() / s_f);
		ref_traj_vel(3) = s_d * (e(3) / s_f); 

		
		ref_traj_acc.head<3>() = s_dd * (e.head<3>() / s_f);
		ref_traj_acc(3) = s_dd * (e(3) / s_f); 
		
		TrajectorySetpoint msg{};
		msg.position = {float(ref_traj_pos(0)), float(ref_traj_pos(1)), float(ref_traj_pos(2))};
		msg.velocity = {float(ref_traj_vel(0)), float(ref_traj_vel(1)), float(ref_traj_vel(2))};
		msg.acceleration = {float(ref_traj_acc(0)), float(ref_traj_acc(1)), float(ref_traj_acc(2))};
		
		msg.yaw = float(ref_traj_pos(3));
		msg.yawspeed = float(ref_traj_vel(3));
		msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

		return msg;
    }
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_local_position listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TrajectoryPlanner>());
	rclcpp::shutdown();
	return 0;
}