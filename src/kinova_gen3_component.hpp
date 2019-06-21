#ifndef OROCOS_KINOVA_GEN3_COMPONENT_HPP
#define OROCOS_KINOVA_GEN3_COMPONENT_HPP

#include <rtt/RTT.hpp> //To include all the elements of a component (e.g. ports, services, etc)
// #include <kdl/frames.hpp>

//All headers needed for establishing the communication with the robot
#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>

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
	bool establish_connection();
	void stop_connection();
	bool check_connection();
	void stream_sensor_info(k_api::BaseCyclic::Feedback BaseFeedback_msg);
	std::string get_all_sensor_jsonstring();
	void set_servoing_mode(int mode);
	bool reach_joint_angles(std::vector<double> value);
	bool reach_cartesian_pose(std::vector<double> value);
	bool change_gripper_aperture(double value);
	std::vector<double> get_joint_angles();
	void start_sending_setpoints();
	void position_low_level_servoing();
	void velocity_low_level_servoing();
	void set_BaseFeedback(k_api::BaseCyclic::Feedback data);


private:
	//Component properties for connection purposes
	int port_number;
	int port_cyclic_number;
	std::string ip_address;

	//Component properties to define the behaviour of the component
	int servoing_mode;
	bool sending_setpoints;

	//Component ports:
	RTT::InputPort<std::vector<double>>      control_joint_positions;
	RTT::InputPort<std::vector<double>>      control_joint_velocities;

	RTT::OutputPort<std::vector<double>>     sensor_joint_angles;
	RTT::OutputPort<std::vector<double>>     sensor_joint_velocities;
	RTT::OutputPort<std::vector<double>>     sensor_joint_torques;
	RTT::OutputPort<std::vector<double>>     tool_pose;
	RTT::OutputPort<std::vector<double>>     tool_twist;
	RTT::OutputPort<std::vector<double>>     tool_external_wrench;
	RTT::OutputPort<std::vector<double>>     tool_imu;
	RTT::OutputPort<std::vector<double>>     gripper_feedback;
	RTT::OutputPort<std::string>     event_port;

	//Class properties needed to communicate with the robot
	bool isConnected;
	k_api::TransportClientUdp* 										pTransport;
	k_api::RouterClient* 													pRouterClient;
	k_api::Base::BaseClient*											pBase;
	k_api::DeviceConfig::DeviceConfigClient* 			pDeviceConfig;
	k_api::SessionManager*												pSessionMng;

//Class properties needed to communicate with the robot for cyclic operations (for low level servoing or to obtain sensor info)
	k_api::TransportClientUdp* 							pTransportRT;
	k_api::RouterClient* 										pRouterClientRT;
	k_api::BaseCyclic::BaseCyclicClient* 		pBaseCyclicRT;
	k_api::SessionManager* 									pSessionMngRT;

	k_api::BaseCyclic::Feedback 						BaseFeedback;
	k_api::BaseCyclic::Command  						BaseCommand;
	k_api::BaseCyclic::Command  						BaseCommandEmpty;

	std::vector<k_api::BaseCyclic::ActuatorCommand> 		ActuatorCommands;
	std::vector<k_api::BaseCyclic::ActuatorFeedback> 		ActuatorFeedbacks;

	//temporary data for sensor information
	std::vector<double> temporary_sensor_data; //for actuator feedback
	std::vector<double> temporary_tool_data; //for tool feedback
	std::vector<double> temporary_gripper_data; //for gripper feedback

	//temporary data for low-level servoing input setpoints
	std::vector<double> temporary_joint_setpoints;
	std::vector<double> integrated_position_setpoints; //Used  temporarily until the API 2.0 is released TODO: Delete
	std::vector<double> initial_angles;
};


#endif
