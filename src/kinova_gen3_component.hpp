// Copyright (C) 2020 Santiago Iregui <santiago.iregui@kuleuven.be>
// src code of an Orocos Component that can be used to control the Kinova Gen3 Ultra lightweight robot.

#ifndef OROCOS_KINOVA_GEN3_COMPONENT_HPP
#define OROCOS_KINOVA_GEN3_COMPONENT_HPP

#include <rtt/RTT.hpp> //To include all the elements of a component (e.g. ports, services, etc)
#include <kdl/frames.hpp>

//All headers needed for establishing the communication with the robot
#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <InterconnectConfigClientRpc.h>

#include <chrono>
#include <thread>

//Different servoing modes
#define HIGH_LEVEL 	0 //By default
#define JOINT_VEL_LOW_LEVEL 	1
#define JOINT_POS_LOW_LEVEL 	2

#define DOF 7


namespace k_api = Kinova::Api;


class kinova_gen3 : public RTT::TaskContext{
public:


	kinova_gen3(std::string const& name);
	bool configureHook();
	bool startHook();
	void updateHook();
	void stopHook();
	void cleanupHook();
	bool breakUpdateHook();
	bool establish_connection();
	void stop_connection();
	bool check_connection();
	void stream_sensor_info(const k_api::BaseCyclic::Feedback &BaseFeedback_msg);
	std::string get_all_sensor_jsonstring();
	void set_servoing_mode(int mode);
	bool reach_joint_angles(std::vector<double> value);
	bool reach_cartesian_pose(std::vector<double> value);
	bool change_gripper_aperture(double value);
	std::vector<double> get_joint_angles();
	void start_sending_setpoints();
	void send_base_command(k_api::BaseCyclic::Command &BaseCommand_msg);
	void position_low_level_servoing();
	void velocity_low_level_servoing();
	void set_BaseFeedback(k_api::BaseCyclic::Feedback data);
	bool check_joint_kinematic_limits();
	bool check_joint_torque_limits(const std::vector<double> &torques);
	void emergency_stop();
	void convert_angle_urdf_convention(double &angle);
	void smoother_linear_interpolation(std::vector<double> &new_setpoints);
	void update_gripper(float &desired_gripper_pos);
	bool change_gripper_aperture_low_level(const double desired_gripper_pos);
	void count_turns(const double &old_angle, const double &new_angle, int &turns);
	void configure_high_level_commands();
	// bool check_joint_kinematic_limits_velocity_based(k_api::BaseCyclic::Feedback BaseFeedback, std::vector<double> velocity_setpoint);


private:
	//Component properties for connection purposes
	int port_number;
	int port_cyclic_number;
	std::string ip_address;
	std::vector<double> velocity_limits;
	std::vector<double> acceleration_limits;
	std::vector<double> torque_limits;
	double setpoints_frequency;
	bool debug_mode;
	bool debug_mode_conf;

	//Component properties to define the behaviour of the component
	int servoing_mode;
	bool sending_setpoints;
	const std::vector<double> position_limits = {1000000,2.41,1000000,2.66,1000000,2.23,1000000};
	float force_gripper;

	//Component ports:
	RTT::InputPort<std::vector<double>>      control_joint_positions;
	RTT::InputPort<std::vector<double>>      control_joint_velocities;

	RTT::OutputPort<std::vector<double>>     sensor_joint_angles;
	RTT::OutputPort<std::vector<double>>     sensor_joint_velocities;
	RTT::OutputPort<std::vector<double>>     sensor_joint_torques;
	RTT::OutputPort<std::vector<double>>     tool_pose;
	RTT::OutputPort<std::vector<double>>     tool_twist;
	RTT::OutputPort<std::vector<double>>     sent_setpoints;
	RTT::OutputPort<KDL::Wrench> 						 tool_external_wrench;
	RTT::OutputPort<double> 								 periodicity; //Only available in debug mode
	RTT::OutputPort<std::vector<double>>     tool_imu;
	RTT::OutputPort<std::vector<double>>     gripper_feedback;
	RTT::OutputPort<std::string>						 event_port;

	//Class properties needed to communicate with the robot
	bool isConnected;
	k_api::TransportClientTcp* 										pTransport;
	k_api::RouterClient* 													pRouterClient;
	k_api::Base::BaseClient*											pBase;
	k_api::DeviceConfig::DeviceConfigClient* 			pDeviceConfig;
	k_api::SessionManager*												pSessionMng;

//Class properties needed to communicate with the robot for cyclic operations (for low level servoing or to obtain sensor info)
	k_api::TransportClientUdp* 																		pTransportRT;
	k_api::RouterClient* 																					pRouterClientRT;
	k_api::BaseCyclic::BaseCyclicClient* 													pBaseCyclicRT;
	k_api::SessionManager* 																				pSessionMngRT;
	k_api::ActuatorConfig::ActuatorConfigClient* 									pActuatorConfig;
	k_api::ActuatorConfig::ControlModeInformation 								pControlModeMessage;

	k_api::BaseCyclic::Feedback 													BaseFeedback;
	k_api::BaseCyclic::Feedback 													BaseFeedbackNotification;
	k_api::BaseCyclic::Command  													BaseCommand;
	k_api::GripperCyclic::MotorCommand*   								GripperCommand;

	//Data for managing periodicity of the driver
	std::chrono::steady_clock::time_point start_time_period;
	std::chrono::steady_clock::time_point end_time_period;
	std::chrono::duration<double,std::milli> elapsed_time_period;
	const std::chrono::nanoseconds period_nano = std::chrono::milliseconds(1);

	std::chrono::steady_clock::time_point end_time_sleep;
	bool begin_counting;


	//temporary data for sensor information
	KDL::Wrench temporary_wrench;
	std::vector<double> temporary_sensor_data; //for actuator feedback
	std::vector<double> temporary_tool_data; //for tool feedback
	std::vector<double> temporary_gripper_data; //for gripper feedback
	std::vector<double> desired_pos; //for safety checking
	std::vector<double> desired_vel; //for safety checking
	std::vector<double> desired_accel; //for safety checking
	std::vector<double> old_desired_pos; //for safety checking
	std::vector<double> smoothed_setpoints; //for smoothing the setpoints
	float virtual_time;
	float error_pos_gripper;
	RTT::FlowStatus type_data; //indicates if the data received is new, old or no data.

	//temporary data for low-level servoing input setpoints
	std::vector<double> temporary_joint_setpoints;
	std::vector<double> integrated_position_setpoints;
	std::vector<double> initial_angles;
	float gripper_initial_position;
	float target_aperture;
	float actual_aperture;
	float target_velocity_gripper;
	float actual_velocity_gripper;
	float actual_current_gripper;
	bool gripper_moving_low_level;
	double timer_gripper;

	//temporary data for check kinematic limits (Safety)
	double angle_urdf_temporary;
	std::vector<double> previous_setpoints;
	std::vector<double> previous_dwells;
	std::vector<double> current_positions;

	//Data for keeping track of the number of turns of the continuous joints
	std::vector<double> old_positions_raw;
	std::vector<int> joints_turns;
	const std::vector<bool> is_continuous_joint = {true,false,true,false,true,false,true}; //Indicates which joints are contiuous

	//Data for handling actions in high level mode
	k_api::Base::Action action_gripper;
	k_api::Base::GripperCommand* gripperCommand;
	k_api::Base::Gripper* gripper;
	k_api::Base::Finger* finger;

	k_api::Base::Action action_cartesian;
	k_api::Base::ConstrainedPose* constrainPose;
	k_api::Base::Pose* pose;

	k_api::Base::Action action_joint;
	k_api::Base::ConstrainedJointAngles* reachJointAngles;
	k_api::Base::JointAngles* jointAngles;
	k_api::Base::JointAngle* jointAngle;

};


#endif
