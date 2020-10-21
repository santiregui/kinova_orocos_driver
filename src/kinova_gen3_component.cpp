// Copyright (C) 2020 Santiago Iregui <santiago.iregui@kuleuven.be>
// src code of an Orocos Component that can be used to control the Kinova Gen3 Ultra lightweight robot.

#include "kinova_gen3_component.hpp"
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>
#include <string>     // std::string, std::to_string
#include <iostream>
#include <math.h>
#include <google/protobuf/util/json_util.h>
#include <KDetailedException.h>


#define DOF 7
#define PERIOD 0.001 //in seconds
#define NSEC_PER_SEC 1000000000l
#define PROPORTIONAL_GAIN_GRIPPER  1.0f

//This lambda function is called by the low level servoing methods. Is defined here to avoid defining it at each iteration.
kinova_gen3* kin_ptr = nullptr;
auto lambda_fct_callback = [&](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
{
    kin_ptr->stream_sensor_info(data);
    // kin_ptr->set_BaseFeedback(data);
    if(err.error_sub_string() != "" ){
      std::cout << "KError error_code: " << err.error_code() << std::endl;
      std::cout << "KError sub_code: " << err.error_sub_code() << std::endl;
      std::cout << "KError sub_string: " << err.error_sub_string() << std::endl;
    }
};

using namespace RTT;
kinova_gen3::kinova_gen3(std::string const& name) : TaskContext(name,PreOperational)
  , port_number(10000)
  , port_cyclic_number(10001)
  , ip_address("192.168.1.10")
  , setpoints_frequency(1000)
  , debug_mode(false)
  , debug_mode_conf(false)
  , servoing_mode(HIGH_LEVEL)
  , sending_setpoints(false)
  , force_gripper(50.0f)
  , begin_counting(false)
  , virtual_time(0.0f)
  , error_pos_gripper(0.0f)
  , type_data(NoData)
  , gripper_initial_position(100)
  , target_velocity_gripper(0.0f)
  , gripper_moving_low_level(false)
  , timer_gripper(0)
  , angle_urdf_temporary(0)
{
kin_ptr = this; //To be used by the lambda function

// this->addProperty("port_number", port_number);
// this->addProperty("port_cyclic_number", port_number).doc("All cyclic operations go through this port");
this->addProperty("ip_address", ip_address).doc("ip address of the kinova robot. It is by default set to the factory IP");
this->addProperty("setpoints_frequency", setpoints_frequency).doc("Frequency at which the low level setpoints are sent");
this->addProperty("debug_mode", debug_mode).doc("If set to true before calling configureHook, additional ports are created to stream data related to timings");
this->addProperty("velocity_limits", velocity_limits).doc("Velocity limits of each joint, used for safety purposes [rad/s]. Set by default to the 'general limits' specified in the user manual");
this->addProperty("acceleration_limits", acceleration_limits).doc("Acceleration limits of each joint, used for safety purposes [rad/s^2]. Set by default to the 'hard limits' specified in the user manual");
this->addProperty("torque_limits", torque_limits).doc("Torque limits of each joint, used for safety purposes [Nm]. Set by default to the 'soft limits' specified in the user manual");
this->addProperty("force_gripper", force_gripper).doc("Percentage of force applied by the gripper when grasping an object. Value from 0 to 100. Used when using change_gripper_aperture_low_level operation.");

Logger::In in(this->getName().data());
log( Info ) << "Kinova driver component has been created" << endlog();

// this->addOperation("check_connection",  &kinova_gen3::check_connection, this, OwnThread).doc("Checks connection with robot");
this->addOperation("set_servoing_mode",  &kinova_gen3::set_servoing_mode, this, OwnThread).doc("Sets the servoing mode to: 0 (HIGH_LEVEL), 1 (JOINT_VEL_LOW_LEVEL) or 2 (JOINT_POS_LOW_LEVEL)");
this->addOperation("get_all_sensor_jsonstring",  &kinova_gen3::get_all_sensor_jsonstring, this, OwnThread).doc("Returns a JSON string with all sensor information");
this->addOperation("reach_joint_angles",  &kinova_gen3::reach_joint_angles, this, OwnThread).doc("Moves the robot to a target position in jointspace (Kinova Controller). Must change to mode = 3 with set_servoing_mode operation");
this->addOperation("reach_cartesian_pose",  &kinova_gen3::reach_cartesian_pose, this, OwnThread).doc("Moves the robot to a target pose (Kinova Controller). Must change to mode = 1 with set_servoing_mode operation");
this->addOperation("change_gripper_aperture",  &kinova_gen3::change_gripper_aperture, this, OwnThread).doc("Change the appertur of the gripper: 1 is completely closed and 0 is completely opened");
this->addOperation("get_joint_angles",  &kinova_gen3::get_joint_angles, this, OwnThread).doc("Get the current sensed joint angles [rad]. Useful to setup your trajectory planner.");
this->addOperation("start_sending_setpoints",  &kinova_gen3::start_sending_setpoints, this, OwnThread).doc("Starts sending the setpoints of the joints to the robot");
this->addOperation("change_gripper_aperture_low_level",  &kinova_gen3::change_gripper_aperture_low_level, this, OwnThread).doc("Change the appertur of the gripper: 1 is completely closed and 0 is completely opened. Use it during low level");

//Input ports (for control signals)
this->addPort( "control_joint_positions", control_joint_positions ).doc( "Input port to pass the joint positions to control the robot [rad]. Will be ignored if the corresponding servoing mode is not configured" );
this->addPort( "control_joint_velocities", control_joint_velocities ).doc( "Input port to pass the joint velocities to control the robot [rad/s]. Will be ignored if the corresponding servoing mode is not configured " );

//Output ports (for feedback)
this->addPort( "sensor_joint_angles", sensor_joint_angles ).doc( "Output Port, to get the sensor joint angles [rad]" );
this->addPort( "sensor_joint_velocities", sensor_joint_velocities ).doc( "Output Port, to get the sensor joint velocities [rad/s]" );
this->addPort( "sensor_joint_torques", sensor_joint_torques ).doc( "Output Port, to get the sensor joint torques [Nm]" );
this->addPort( "tool_pose", tool_pose ).doc( "Output Port, Gets the tool pose calculated from the measured joint angles: three positions [m] and three orientations [rad]" );
this->addPort( "tool_twist", tool_twist ).doc( "Output Port, Gets the tool twist calculated from the measured data: three linear velocities [m/s] and three orientation velocities [rad/s] expressed in the global frame" );
this->addPort( "tool_external_wrench",tool_external_wrench ).doc( "(For a future release of the Kortex API) Output Port, Gets the tool external wrench calculated from the measured data: three forces [N] and three moments [Nm]" );
this->addPort( "tool_imu",tool_imu ).doc( "Output Port, Gets the information from the IMU: First three linear accelerations [m/s^2] and then three angular velocities [rad/s]" );
this->addPort( "gripper_feedback",gripper_feedback ).doc( "Output Port, Gets the information from the Gripper: position (0-100), velocity (0-100), current (mA)" );
this->addPort( "sent_setpoints", sent_setpoints ).doc( "Output Port, to get the setpoints sent to the robot" );

//Event output port
this->addPort( "event_port",event_port ).doc( "Gets events for the high-level servoing mode (when actions are complete). Events: gripper_done_no_obj, gripper_done_with_obj, joints_done, cart_done" );

//Memory allocation for the size of the vectors done beforehand (real time)

start_time_period = std::chrono::steady_clock::now();
end_time_period = std::chrono::steady_clock::now();
elapsed_time_period = end_time_period-start_time_period;

temporary_sensor_data.resize(DOF,0.0);
temporary_tool_data.resize(6,0.0);
temporary_gripper_data.resize(3,0.0);
temporary_joint_setpoints.resize(DOF,0.0);
integrated_position_setpoints.resize(DOF,0.0);
initial_angles.resize(DOF,0.0);
desired_pos.resize(DOF,0.0);
desired_vel.resize(DOF,0.0);
desired_accel.resize(DOF,0.0);
old_desired_pos.resize(DOF,0.0);
smoothed_setpoints.resize(DOF,0.0);

velocity_limits = {1.3963, 1.3963,1.3963,1.3963,1.2218,1.2218,1.2218};
acceleration_limits = {5.2 , 5.2 ,5.2 ,5.2 ,10.0 ,10.0 ,10.0 };
torque_limits = {39.0 , 39.0 ,39.0 ,39.0 ,9.0 ,9.0 ,9.0};

previous_setpoints.resize(DOF,0.0);
previous_dwells.resize(DOF,0.0);
current_positions.resize(DOF,0.0);

old_positions_raw.resize(DOF,0.0);
joints_turns.resize(DOF,0);

}


bool kinova_gen3::establish_connection(){

try{


  auto errorCallback = [](k_api::KError err){ log( Error ) << "Error while establishing the connection with the robot (BaseClient):   "<< err.toString() << endlog(); };
  auto errorCallbackRT = [](k_api::KError err) { log( Error ) << "Error while establishing the connection with the robot (BaseCyclicClient):   "<< err.toString() << endlog(); };

  pTransport = new k_api::TransportClientTcp();
  pTransportRT = new k_api::TransportClientUdp();

  pTransport->connect(ip_address, port_number);
  pTransportRT->connect(ip_address, port_cyclic_number);

  pRouterClient = new k_api::RouterClient(pTransport, errorCallback);
  pRouterClientRT = new k_api::RouterClient(pTransportRT, errorCallbackRT);

  auto createSessionInfo = k_api::Session::CreateSessionInfo();
  createSessionInfo.set_username("admin");
  createSessionInfo.set_password("admin");
  createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
  createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

  auto createSessionInfoRT = k_api::Session::CreateSessionInfo();
  createSessionInfoRT.set_username("admin");
  createSessionInfoRT.set_password("admin");
  createSessionInfoRT.set_session_inactivity_timeout(60000);    // (milliseconds)
  createSessionInfoRT.set_connection_inactivity_timeout(2000);  // (milliseconds)

  pSessionMng = new k_api::SessionManager(pRouterClient);
  pSessionMngRT = new k_api::SessionManager(pRouterClientRT);

  pSessionMng->CreateSession(createSessionInfo);
  pSessionMngRT->CreateSession(createSessionInfoRT);

  pDeviceConfig = new k_api::DeviceConfig::DeviceConfigClient(pRouterClient);

  pBase = new k_api::Base::BaseClient(pRouterClient);
  pBaseCyclicRT = new k_api::BaseCyclic::BaseCyclicClient(pRouterClientRT);

  pActuatorConfig = new k_api::ActuatorConfig::ActuatorConfigClient(pRouterClient);
  pControlModeMessage = k_api::ActuatorConfig::ControlModeInformation();

  isConnected = true;
  log( Info ) << "The client/server connection with the Kinova robot has been established" << endlog();

}
catch(k_api::KDetailedException& ex)
{
  isConnected = false;
    auto errorInfo = ex.getErrorInfo();
    auto errorCode = errorInfo.getError();
    log( Error ) << "The client/server connection with the Kinova robot could not be established" << endlog();
    log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
    log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

    log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
    log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
    log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
    return false;
}
catch (...) {
  isConnected = false;
  log( Error ) << "The client/server connection with the Kinova robot could not be established" << endlog();
  return false;
}
  return true;
}

void kinova_gen3::stop_connection(){

  if(isConnected) {
    try{
      std::cout << "Stoping the connection with the robot!" << std::endl;
      set_servoing_mode(HIGH_LEVEL);

      pControlModeMessage.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION); //Just in case in the future we change the control mode
      for(int i = 0; i < DOF; i++)
      {
        pActuatorConfig->SetControlMode(pControlModeMessage, i+1); //The index of the actuator apparently is not zero-based
      }

      pSessionMng->CloseSession();
      // Destroy the API
      pTransport->disconnect();
      delete pBase;
      delete pDeviceConfig;
      delete pSessionMng;
      delete pRouterClient;
      delete pTransport;
      delete pTransportRT;
      delete pRouterClientRT;
      delete pBaseCyclicRT;
      delete pSessionMngRT;
      delete pActuatorConfig;
      isConnected = false;
    }
    catch(k_api::KDetailedException& ex)
    {
        auto errorInfo = ex.getErrorInfo();
        auto errorCode = errorInfo.getError();
        log( Error ) << "Could not stop the connection with the robot." << endlog();
        log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
        log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

        log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
        log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
        log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
    }
    catch(k_api::KBasicException& ex) {
      log( Error ) << "Could not stop the connection with the robot." << endlog();
      log( Error ) << "KBasicException detected toStr: " << ex.toString().c_str() << endlog();
      log( Error ) << "KDBasicdoption detected what:  " << ex.what() << endlog();
    }
    catch(...){
      log( Error ) << "Could not stop the connection with the robot." << endlog();
    }
  }
  else {
    log( Warning ) << "The robot was not connected anyway" << endlog();
  }
}



bool kinova_gen3::check_connection()
{
//TODO: Implement a better check
  return isConnected;
}



void kinova_gen3::stream_sensor_info(const k_api::BaseCyclic::Feedback &BaseFeedback_msg) {

  //The sensor info list can be found here: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/references/msg_BaseCyclic_ActuatorFeedback.md#
  actual_aperture = BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].position();
  actual_velocity_gripper = BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].velocity();
  actual_current_gripper = BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].current_motor();
  if ( sensor_joint_angles.connected() ){
    for (unsigned int i = 0; i < DOF; i++) {
        temporary_sensor_data[i] =  BaseFeedback_msg.actuators(i).position()*3.1415926/180;

        if (is_continuous_joint[i]){
          count_turns(old_positions_raw[i], temporary_sensor_data[i],joints_turns[i]);
          temporary_sensor_data[i] = temporary_sensor_data[i] + joints_turns[i]*2*3.1415926;
          // temporary_sensor_data[i] = fmod(temporary_sensor_data[i], 2*3.1415926) + joints_turns[i]*2*3.1415926;
        }
        else{
          convert_angle_urdf_convention(temporary_sensor_data[i]);
        }



        // if(temporary_sensor_data[i] > 3.1415926){
        //   temporary_sensor_data[i] = temporary_sensor_data[i] - 2*3.1415926;
        // }
        // else if(temporary_sensor_data[i] < -3.1415926){
        //   temporary_sensor_data[i] = temporary_sensor_data[i] + 2*3.1415926;
        // }
        current_positions[i] =  temporary_sensor_data[i];
        old_positions_raw[i] =  BaseFeedback_msg.actuators(i).position()*3.1415926/180;
    }
    sensor_joint_angles.write(temporary_sensor_data);
    // std::fill(temporary_sensor_data.begin(), temporary_sensor_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.

  }
  if ( sensor_joint_velocities.connected() ){
    for (unsigned int i = 0; i < DOF; i++) {
        temporary_sensor_data[i] =  BaseFeedback_msg.actuators(i).velocity()*3.1415926/180;
    }
    sensor_joint_velocities.write(temporary_sensor_data);
    // std::fill(temporary_sensor_data.begin(), temporary_sensor_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.

  }

  if ( tool_pose.connected() ){
    temporary_tool_data[0] =  BaseFeedback_msg.base().tool_pose_x();
    temporary_tool_data[1] =  BaseFeedback_msg.base().tool_pose_y();
    temporary_tool_data[2] =  BaseFeedback_msg.base().tool_pose_z();
    temporary_tool_data[3] =  BaseFeedback_msg.base().tool_pose_theta_x()*3.1415926/180;
    temporary_tool_data[4] =  BaseFeedback_msg.base().tool_pose_theta_y()*3.1415926/180;
    temporary_tool_data[5] =  BaseFeedback_msg.base().tool_pose_theta_z()*3.1415926/180;

    tool_pose.write(temporary_tool_data);
    // std::fill(temporary_tool_data.begin(), temporary_tool_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.

  }
  if ( tool_twist.connected() ){
    temporary_tool_data[0] =  BaseFeedback_msg.base().tool_twist_linear_x();
    temporary_tool_data[1] =  BaseFeedback_msg.base().tool_twist_linear_y();
    temporary_tool_data[2] =  BaseFeedback_msg.base().tool_twist_linear_z();
    temporary_tool_data[3] =  BaseFeedback_msg.base().tool_twist_angular_x()*3.1415926/180;
    temporary_tool_data[4] =  BaseFeedback_msg.base().tool_twist_angular_y()*3.1415926/180;
    temporary_tool_data[5] =  BaseFeedback_msg.base().tool_twist_angular_z()*3.1415926/180;

    tool_twist.write(temporary_tool_data);
    // std::fill(temporary_tool_data.begin(), temporary_tool_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.
  }
  if ( tool_external_wrench.connected() ){

    temporary_wrench.force.x((double)BaseFeedback_msg.base().tool_external_wrench_force_x());
    temporary_wrench.force.y((double)BaseFeedback_msg.base().tool_external_wrench_force_y());
    temporary_wrench.force.z((double)BaseFeedback_msg.base().tool_external_wrench_force_z());
    temporary_wrench.torque.x((double)BaseFeedback_msg.base().tool_external_wrench_torque_x());
    temporary_wrench.torque.y((double)BaseFeedback_msg.base().tool_external_wrench_torque_y());
    temporary_wrench.torque.z((double)BaseFeedback_msg.base().tool_external_wrench_torque_z());
    tool_external_wrench.write(temporary_wrench);

    // temporary_tool_data[0] =  (double)BaseFeedback_msg.base().tool_external_wrench_force_x();
    // temporary_tool_data[1] =  (double)BaseFeedback_msg.base().tool_external_wrench_force_y();
    // temporary_tool_data[2] =  (double)BaseFeedback_msg.base().tool_external_wrench_force_z();
    // temporary_tool_data[3] =  (double)BaseFeedback_msg.base().tool_external_wrench_torque_x();
    // temporary_tool_data[4] =  (double)BaseFeedback_msg.base().tool_external_wrench_torque_y();
    // temporary_tool_data[5] =  (double)BaseFeedback_msg.base().tool_external_wrench_torque_z();
    // tool_external_wrench.write(temporary_tool_data);
    // std::fill(temporary_tool_data.begin(), temporary_tool_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.
  }
  if ( tool_imu.connected() ){
    temporary_tool_data[0] =  BaseFeedback_msg.interconnect().imu_acceleration_x();
    temporary_tool_data[1] =  BaseFeedback_msg.interconnect().imu_acceleration_y();
    temporary_tool_data[2] =  BaseFeedback_msg.interconnect().imu_acceleration_z();
    temporary_tool_data[3] =  BaseFeedback_msg.interconnect().imu_angular_velocity_x()*3.1415926/180;
    temporary_tool_data[4] =  BaseFeedback_msg.interconnect().imu_angular_velocity_y()*3.1415926/180;
    temporary_tool_data[5] =  BaseFeedback_msg.interconnect().imu_angular_velocity_z()*3.1415926/180;

    tool_imu.write(temporary_tool_data);
    // std::fill(temporary_tool_data.begin(), temporary_tool_data.end(), 0.0); //Assigns zero values to the vector, just in case to prevent the posibility of observing old values. Can be deleted to improve a bit the speed.
  }
  if ( gripper_feedback.connected() ){
    temporary_gripper_data[0] =  BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].position()/100;
    temporary_gripper_data[1] =  BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].velocity()/100;
    temporary_gripper_data[2] =  BaseFeedback_msg.interconnect().gripper_feedback().motor()[0].current_motor();

    gripper_feedback.write(temporary_gripper_data);

  // std::string serializedData;
  // google::protobuf::util::MessageToJsonString(BaseFeedback_msg.interconnect(), &serializedData);
  // std::cout << serializedData << std::endl;
  }
  for (unsigned int i = 0; i < DOF; i++) {
    temporary_sensor_data[i] =  BaseFeedback_msg.actuators(i).torque();
  }
  if ( sensor_joint_torques.connected() ){
    sensor_joint_torques.write(temporary_sensor_data);
  }
  if (!check_joint_torque_limits(temporary_sensor_data)){
    emergency_stop();
  }
}


std::string kinova_gen3::get_all_sensor_jsonstring(){
  std::string serializedData;
  BaseFeedback = pBaseCyclicRT->RefreshFeedback();
  google::protobuf::util::MessageToJsonString(BaseFeedback, &serializedData);
  return serializedData;
}

std::vector<double> kinova_gen3::get_joint_angles(){
  BaseFeedback = pBaseCyclicRT->RefreshFeedback();
  for (unsigned int i = 0; i < DOF; i++) {
      temporary_sensor_data[i] =  BaseFeedback.actuators(i).position()*3.1415926/180;

      if (is_continuous_joint[i]){
        temporary_sensor_data[i] = temporary_sensor_data[i] + joints_turns[i]*2*3.1415926;
      }
      else{
        convert_angle_urdf_convention(temporary_sensor_data[i]);
      }

  }

  return temporary_sensor_data;
}


void kinova_gen3::set_servoing_mode(int mode) {

  auto servoingMode = k_api::Base::ServoingModeInformation();
  sending_setpoints = false;
  servoing_mode = mode;

  switch (mode) {

  case HIGH_LEVEL:
  {
      initial_angles = get_joint_angles();
      servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      pBase->SetServoingMode(servoingMode);
    }
    break;

  case JOINT_POS_LOW_LEVEL:
    {
      servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      pBase->SetServoingMode(servoingMode);
      // pControlModeMessage.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      // for(int i = 0; i < DOF; i++)
      // {
      //   pActuatorConfig->SetControlMode(pControlModeMessage, i+1); //The index of the actuator apparently is not zero-based
      // }
    }
    break;

  case JOINT_VEL_LOW_LEVEL:
  {
      servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING	);
      pBase->SetServoingMode(servoingMode);
      // Right now the velocity controller of kinova is not suitable to be used (when sending zero velocities the robot still moves slowly due to gravity. The virtual emergency stop also doesn't work in this mode)
      // pControlModeMessage.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);

      // pControlModeMessage.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
      // for(int i = 0; i < DOF; i++)
      // {
      //   pActuatorConfig->SetControlMode(pControlModeMessage, i+1); //The index of the actuator apparently is not zero-based
      // }

    }
    break;

  default: // compilation error: jump to default: would enter the scope of 'x'
    // without initializing it
    {
    initial_angles = get_joint_angles();
    servoing_mode = HIGH_LEVEL;
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    pBase->SetServoingMode(servoingMode);
    log(Error) << "The value:  " << mode
           << "   is not a valid number. For safety, the servoing mode has been set to 0 (HIGH_LEVEL). Remember: HIGH_LEVEL = 0, JOINT_VEL_LOW_LEVEL = 1, JOINT_POS_LOW_LEVEL = 2" << endlog();
    }
   break;
  }
}

void kinova_gen3::send_base_command(k_api::BaseCyclic::Command &BaseCommand_msg)
{
  try
  {
    //Uncomment to debug (very useful)
    // std::string serializedData;
    // google::protobuf::util::MessageToJsonString(BaseCommand, &serializedData);
    // std::cout << serializedData << std::endl;
    BaseCommand_msg.set_frame_id(BaseCommand_msg.frame_id() + 1);
    if (BaseCommand_msg.frame_id() > 65535){
      BaseCommand_msg.set_frame_id(0);
    } // Incrementing identifier ensures actuators can reject out of time frames
    for(int i = 0; i < DOF; i++)
    {
      BaseCommand_msg.mutable_actuators(i)->set_command_id(BaseCommand_msg.frame_id());
    }
    update_gripper(target_aperture);
    pBaseCyclicRT->Refresh_callback(BaseCommand_msg, lambda_fct_callback);

    if(type_data == NewData){
      if (!check_joint_kinematic_limits()){
        emergency_stop();
      }
      previous_setpoints = temporary_joint_setpoints;
    }
  }
  catch(k_api::KDetailedException& ex){
      auto errorInfo = ex.getErrorInfo();
      auto errorCode = errorInfo.getError();
      log( Error ) << "Failed to send the command to the robot. The component has stopped. " << endlog();
      log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
      log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

      log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
      log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
      log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
      emergency_stop();
      this->stop();
  }
  catch (std::runtime_error& ex2)
  {
    log( Error ) << "Failed to send the command to the robot. The component has stopped. " << endlog();
    log( Error ) << "Error: " << ex2.what() << endlog();
    emergency_stop();
    this->stop();
  }
  catch(...)
  {
    log( Error ) << "Failed to send the command to the robot. The component has stopped. " << endlog();
    emergency_stop();
    this->stop();
  }
}

void kinova_gen3::position_low_level_servoing()
{
  if(servoing_mode == JOINT_POS_LOW_LEVEL) {
    if(control_joint_positions.connected()){
      type_data = control_joint_positions.read(temporary_joint_setpoints);

      if ( type_data != NoData ){
        if (temporary_joint_setpoints.size()!=7){
          log(Error)<<this->getName()<<": error size of data in port "<<control_joint_positions.getName()<<".\n STOPPING."<< endlog();
          this->stop();
        }
        else{
          smoother_linear_interpolation(temporary_joint_setpoints); //Modifies the smoothed_setpoints
          for(int i = 0; i < DOF; i++)
          {
            BaseCommand.mutable_actuators(i)->set_position(fmod((smoothed_setpoints[i]*180/3.1415926), 360.0f));
          }
          if(sent_setpoints.connected()){
            sent_setpoints.write(smoothed_setpoints);
          }
          send_base_command(BaseCommand);
        }
      }
    }
    else{
      log( Error ) << "Port control_joint_positions is not connected. The component has stopped. " << endlog();
      emergency_stop();
      this->stop();
    }
  }
  else {
    log( Error ) << "Use set_servoing_mode operation first to set the servoing_mode to 2 (JOINT_POS_LOW_LEVEL) servoing mode. The component has stopped" << endlog();
    this->stop();
  }

}



void kinova_gen3::velocity_low_level_servoing()
{
  if(servoing_mode == JOINT_VEL_LOW_LEVEL) {
    if(control_joint_velocities.connected()){
      type_data = control_joint_velocities.read(temporary_joint_setpoints);

      if ( type_data != NoData ){
        if (temporary_joint_setpoints.size()!=7){
          log(Error)<<this->getName()<<": error size of data in port "<<control_joint_velocities.getName()<<".\n STOPPING."<< endlog();
          this->stop();
        }
        else{
          smoother_linear_interpolation(temporary_joint_setpoints); //Modifies the smoothed_setpoints
          for(int i = 0; i < DOF; i++)
          {
            // BaseCommand.mutable_actuators(i)->set_position(BaseFeedback.actuators(i).position()); //This is used by Kinova to check the "following error" (search for it in the manual).
            // BaseCommand.mutable_actuators(i)->set_velocity(smoothed_setpoints[i]*180/3.1415926);
            integrated_position_setpoints[i] = integrated_position_setpoints[i] + smoothed_setpoints[i]*PERIOD;
            BaseCommand.mutable_actuators(i)->set_position(fmod((integrated_position_setpoints[i]*180/3.1415926), 360.0f));
          }
          if(sent_setpoints.connected()){
            sent_setpoints.write(smoothed_setpoints);
          }
          send_base_command(BaseCommand);
        }
      }
    }
    else{
      log( Error ) << "Port control_joint_velocities is not connected. The component has stopped. " << endlog();
      emergency_stop();
      this->stop();
    }
  }
  else {
    log( Error ) << "Use set_servoing_mode operation first to set the servoing_mode to 1 (JOINT_VEL_LOW_LEVEL) servoing mode. The component has stopped" << endlog();
    this->stop();
  }

}

bool kinova_gen3::reach_joint_angles(std::vector<double> value)
{
  if(check_connection()){
    if(servoing_mode == HIGH_LEVEL){

      for(unsigned int i = 0 ; i < DOF; ++i)
      {
          jointAngle = jointAngles->mutable_joint_angles(i);
          jointAngle->set_value((float)value.at(i)*180/3.1415926);
      }
      try{
        pBase->ExecuteAction(action_joint);
      }
      catch(k_api::KDetailedException& ex) {
          auto errorInfo = ex.getErrorInfo();
          auto errorCode = errorInfo.getError();
          log( Error ) << "Could not execute the action to reach the joint angles" << endlog();
          log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
          log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

          log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
          log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
          log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
          return false;
      }
      catch(k_api::KBasicException& ex) {
        log( Error ) << "Could not execute the action to reach the joint angles" << endlog();
        log( Error ) << "KBasicException detected toStr: " << ex.toString().c_str() << endlog();
        log( Error ) << "KDBasicdoption detected what:  " << ex.what() << endlog();
        return false;
      }
      catch(...){
        log( Error ) << "Could not execute the action to reach the joint angles" << endlog();
        return false;
      }
      return true;
    }
    else{
      log( Warning ) << "First you need to set the servoing mode to 0 (HIGH_LEVEL) with set_servoing_mode operation." << endlog();
      return false;
    }
  }
  else{
    log( Warning ) << "The connection with the robot has not been established or it was interrumpted." << endlog();
    return false;
  }
}

bool kinova_gen3::reach_cartesian_pose(std::vector<double> value)
{

  if(check_connection()){
    if(servoing_mode == HIGH_LEVEL){

      pose->set_x((float) value.at(0));           // x (meters)
      pose->set_y((float) value.at(1));           // y (meters)
      pose->set_z((float) value.at(2));          // z (meters)
      pose->set_theta_x((float) value.at(3)*180/3.1415926);    // theta x (degrees)
      pose->set_theta_y((float) value.at(4)*180/3.1415926);    // theta y (degrees)
      pose->set_theta_z((float) value.at(5)*180/3.1415926);    // theta z (degrees)

      try{
        pBase->ExecuteAction(action_cartesian);
      }
      catch(k_api::KDetailedException& ex){

          auto errorInfo = ex.getErrorInfo();
          auto errorCode = errorInfo.getError();
          log( Error ) << "Could not execute the action to reach the pose" << endlog();
          log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
          log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

          log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
          log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
          log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
          return false;
      }
      catch(k_api::KBasicException& ex) {
        log( Error ) << "Could not execute the action to reach the pose" << endlog();
        log( Error ) << "KBasicException detected toStr: " << ex.toString().c_str() << endlog();
        log( Error ) << "KDBasicdoption detected what:  " << ex.what() << endlog();
        return false;
      }
      catch(...){
        log( Error ) << "Could not execute the action to reach the pose" << endlog();
        return false;
      }
      return true;
    }
    else{
      log( Warning ) << "First you need to set the servoing mode to 1 (HIGH_LEVEL) with set_servoing_mode operation." << endlog();
      return false;
    }
  }
  else{
    log( Warning ) << "The connection with the robot has not been established or it was interrumpted." << endlog();
    return false;
  }
}

bool kinova_gen3::change_gripper_aperture(double value)
{

  if(servoing_mode == HIGH_LEVEL){
    if(check_connection()){
      finger->set_value(value);
      pBase->ExecuteAction(action_gripper);
      return true;
    }
    else{
      log( Warning ) << "The connection with the robot has not been established or it was interrumpted." << endlog();
      return false;
    }
  }
  else{
    log( Warning ) << "This operation just works for high level servoing mode. For low level use change_gripper_aperture_low_level operation." << endlog();
    return false;
  }

}

void kinova_gen3::start_sending_setpoints()
{
  if (this->getPeriod() != 0){
    log( Error ) << "The driver needs to be non-periodic to operate. Please change the periodicity of the activity to 0 and run again." << this->getPeriod() << endlog();
    return;
  }
  try{
    if(servoing_mode == JOINT_VEL_LOW_LEVEL || servoing_mode == JOINT_POS_LOW_LEVEL){
      BaseFeedback = pBaseCyclicRT->RefreshFeedback(); //TODO:Try to replace for RefreshCustomData in order to reduce the amount of data transferred.
      stream_sensor_info(BaseFeedback);
      initial_angles=get_joint_angles();

      for(int i = 0; i < DOF; i++)
      {

        integrated_position_setpoints[i] = initial_angles[i];
        old_desired_pos[i] = initial_angles[i];
        BaseCommand.mutable_actuators(i)->set_position(BaseFeedback.actuators(i).position());
        if(servoing_mode == JOINT_POS_LOW_LEVEL){
          previous_setpoints[i] = initial_angles[i];
        }
      }
      // BaseFeedback = pBaseCyclicRT->Refresh(BaseCommand); //Aparently necesary for the robot to know where it is, I'm not sure why

      sending_setpoints = true;
    }
    else{
      log( Warning ) << "This operation does not work during high level servoing mode." << endlog();
    }
  }
  catch(k_api::KDetailedException& ex){
      auto errorInfo = ex.getErrorInfo();
      auto errorCode = errorInfo.getError();
      log( Error ) << "Could not read sensor information" << endlog();
      log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
      log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

      log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
      log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
      log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
      return;
  }
  catch(...){
    log( Error ) << "Could not read sensor information" << endlog();
    return;
  }


}

bool kinova_gen3::check_joint_kinematic_limits()
{
  if (type_data == NewData){
    for (unsigned int i = 0; i < DOF; i++) {

        if (servoing_mode == JOINT_POS_LOW_LEVEL){ //For position setpoints
          desired_pos[i] = temporary_joint_setpoints[i];
          desired_vel[i] =  (desired_pos[i] - old_desired_pos[i])/(1/setpoints_frequency);
          old_desired_pos[i] = desired_pos[i];
        }
        else if (servoing_mode == JOINT_VEL_LOW_LEVEL){
          desired_pos[i] = old_desired_pos[i] + temporary_joint_setpoints[i]*(1/setpoints_frequency);
          desired_vel[i] =  temporary_joint_setpoints[i];
          desired_accel[i] =  (desired_vel[i] - previous_setpoints[i])/(1/setpoints_frequency);
          old_desired_pos[i] = current_positions[i];
          if (fabs(desired_accel[i]) > acceleration_limits[i]){
            log( Error ) << "The desired position setpoint violates the specified acceleration limit of joint #" << i+1 << " with a value of " << desired_accel[i] << "rad/s^2"<< endlog();
            return false;
          }
        }

        if (fabs(desired_pos[i]) > position_limits[i]){
          log( Error ) << "The desired position setpoint violates the position limit of joint #" << i+1 << " with a value of " << desired_pos[i] << "rad"<< endlog();
          return false;
        }
        if (fabs(desired_vel[i]) > velocity_limits[i]){
          log( Error ) << "The desired position setpoint violates the specified velocity limit of joint #" << i+1 << " with a value of " << desired_vel[i] << "rad/s"<< endlog();
          return false;
        }
    }
  }

  return true;
}

bool kinova_gen3::check_joint_torque_limits(const std::vector<double> &torques)
{
  for (unsigned int i = 0; i < DOF; i++) {
    if (fabs(torques[i]) > torque_limits[i]){
      log( Error ) << "The torque of joint #" << i+1 << " with a value of " << torques[i] << " Nm is above the threashold (" << torque_limits[i] << " Nm). \n Check if a collision occurred, and if not you can increase the torque safety limit if your application requires it (orocos property)"<< endlog();
      return false;
    }
  }

  return true;
}


void kinova_gen3::emergency_stop()
{
  pBase->ApplyEmergencyStop();
  sending_setpoints = false;
}

void  kinova_gen3::convert_angle_urdf_convention(double &angle)
{
  if(angle > 3.1415926){
    angle = angle - 2*3.1415926;
  }
  else if(angle < -3.1415926){ //Just in case, but probably will never happen since the angle in the feedback is defined from 0 to 360 (as in the web interface)
    angle = angle + 2*3.1415926;
  }
}

void kinova_gen3::set_BaseFeedback(k_api::BaseCyclic::Feedback data){
  BaseFeedback = data;
}

void kinova_gen3::count_turns(const double &old_angle, const double &new_angle,int &turns){
  if (old_angle > 2*3.14 && new_angle < 0.003){
    turns+=1;
  }
  else if (old_angle < 0.003 && new_angle > 2*3.14){
    turns-=1;
  }
}


void  kinova_gen3::smoother_linear_interpolation(std::vector<double> &new_setpoints)
{
  if(type_data == NewData){
    // std::cout <<  virtual_time << std::endl;
    virtual_time = 0.0f;

    for (unsigned int i = 0; i < DOF; i++) {
      previous_dwells[i] = previous_setpoints[i];
    }
  }

  if ((1/setpoints_frequency) <= PERIOD || virtual_time >= 1){
    for (unsigned int i = 0; i < DOF; i++) {
      smoothed_setpoints[i] = new_setpoints[i];
    }
  }
  else{
    virtual_time=virtual_time + (PERIOD*setpoints_frequency);

    for (unsigned int i = 0; i < DOF; i++) {
      smoothed_setpoints[i] = (new_setpoints[i]-previous_dwells[i])*virtual_time+previous_dwells[i];
    }
  }

}

bool kinova_gen3::change_gripper_aperture_low_level(const double desired_gripper_pos)
{
  if (desired_gripper_pos > 1.0 || desired_gripper_pos < 0.0)
  {
      log( Warning ) << "Invalid desired aperture: the target aperture must be between 0 and 1." << endlog();
      return false;
  }

  target_aperture = (float)desired_gripper_pos*100;
  gripper_moving_low_level = true;
  return true;
}

void kinova_gen3::update_gripper(float &desired_gripper_pos)
{

    try
    {
        error_pos_gripper = desired_gripper_pos - actual_aperture;

        // std::cout << " current : " << actual_current_gripper << std::endl;
        if (gripper_moving_low_level)
        {
          if (fabs(error_pos_gripper) < 1)
          {
            timer_gripper = 0;
            event_port.write("gripper_done_no_obj");
            gripper_moving_low_level = false;
            desired_gripper_pos = actual_aperture;
          }
          else if (timer_gripper>0.1 && fabs(actual_velocity_gripper)<0.001)//Need a timer because in the beginning of the movement the velocity is zero
          {
            timer_gripper = 0;
            event_port.write("gripper_done_with_obj");
            gripper_moving_low_level = false;
            desired_gripper_pos = actual_aperture;
          }
          else
          {
            timer_gripper+=PERIOD;
          }
        }

        target_velocity_gripper = PROPORTIONAL_GAIN_GRIPPER * fabs(error_pos_gripper);
        if (target_velocity_gripper > 100.0)
        {
            target_velocity_gripper = 100.0;
        }

        GripperCommand->set_position(desired_gripper_pos);
        GripperCommand->set_velocity(target_velocity_gripper);
    }
    catch(const std::exception& ex)
    {
        log( Error ) << "Error occurred when trying to open the gripper: " << ex.what() << endlog();
    }
}


bool kinova_gen3::configureHook(){

    establish_connection();

    if(isConnected){
      // Makes the port to send data of the size of temporary_sensor_data (size = 7)
      sensor_joint_angles.setDataSample( temporary_sensor_data );
      sensor_joint_velocities.setDataSample( temporary_sensor_data );
      sensor_joint_torques.setDataSample( temporary_sensor_data );
      sent_setpoints.setDataSample( temporary_sensor_data );

      tool_pose.setDataSample( temporary_tool_data );
      tool_twist.setDataSample( temporary_tool_data );
      tool_external_wrench.setDataSample( temporary_wrench );
      // tool_external_wrench.setDataSample( temporary_tool_data );
      tool_imu.setDataSample( temporary_tool_data );
      gripper_feedback.setDataSample( temporary_gripper_data );
      event_port.setDataSample( "gripper_done_with_obj" );

      if (debug_mode) {
        debug_mode_conf = debug_mode;
        this->addPort( "periodicity", periodicity ).doc( "Time elapsed since the previous loop execution (ideally should be 0.001)" );
        periodicity.setDataSample( (double)elapsed_time_period.count() );
      }

      BaseFeedback = pBaseCyclicRT->RefreshFeedback(); //Sends the initial position

      for (unsigned int i = 0; i < DOF; i++) {
          temporary_sensor_data[i] =  BaseFeedback.actuators(i).position()*3.1415926/180;
          old_positions_raw[i] = temporary_sensor_data[i];

          if (is_continuous_joint[i]){
            if(temporary_sensor_data[i] > 3.1415926){
              joints_turns[i] = -1; //This converts the convention of the initial angle to be from -pi to pi, as the urdf (done to avoid confusions)
            }
            temporary_sensor_data[i] = temporary_sensor_data[i] + joints_turns[i]*2*3.1415926;
          }
          else{
            convert_angle_urdf_convention(temporary_sensor_data[i]);
          }
          initial_angles[i] = temporary_sensor_data[i];
      }

      stream_sensor_info(BaseFeedback);

      configure_high_level_commands();


    }
    else{
      log( Error ) << "configureHook: No connection can be stablished with the robot. The component cannot be configured" << endlog();
    }

		return true;
	}

  void kinova_gen3::configure_high_level_commands()
  {
    // Configure Gripper movement
    action_gripper = k_api::Base::Action();
    action_gripper.set_name("Gripper movement");
    action_gripper.set_application_data("");
    gripperCommand = action_gripper.mutable_send_gripper_command();
    gripperCommand->set_mode(Kinova::Api::Base::GRIPPER_POSITION);
    gripper = gripperCommand->mutable_gripper();
    finger = gripper->add_finger();
    finger->set_finger_identifier(1);

    //Configure Cartesian movement
    action_cartesian = k_api::Base::Action();
    action_cartesian.set_name("Cartesian movement");
    action_cartesian.set_application_data("");
    constrainPose = action_cartesian.mutable_reach_pose();
    pose = constrainPose->mutable_target_pose();

    //Configure Angular movement
    action_joint = k_api::Base::Action();
    action_joint.set_name("Angular movement");
    action_joint.set_application_data("");
    reachJointAngles = action_joint.mutable_reach_joint_angles();
    jointAngles = reachJointAngles->mutable_joint_angles();
    for(unsigned int i = 0 ; i < DOF; ++i)
    {
        jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
    }
  }


/**
 * This function is for the application's start up code.
 * Return false to abort start up.
 */
bool kinova_gen3::startHook() {

  if(!isConnected){
    log( Error ) << "startHook: No connection can be stablished with the robot. The component cannot start its execution" << endlog();
    this->stop();
    return false;
  }
  else{
    if(!sensor_joint_angles.connected() && !sensor_joint_velocities.connected() && !sensor_joint_torques.connected()){
      log( Warning ) << "None of the feedback ports (sensor_joint_angles or sensor_joint_velocities or sensor_joint_torques) has been connected" << endlog();
    }
    else{
      if ( !sensor_joint_angles.connected() ){
        log( Info ) << "The port sensor_joint_angles is not connected yet" << endlog();
      }
      if ( !sensor_joint_velocities.connected() ){
        log( Info ) << "The port sensor_joint_velocities is not connected yet" << endlog();
      }
      if ( !sensor_joint_torques.connected() ){
        log( Info ) << "The port sensor_joint_torques is not connected yet" << endlog();
      }
    }
    initial_angles = get_joint_angles();

    for(int i = 0; i < DOF; i++)
    {

      old_desired_pos[i] = initial_angles[i];
      BaseCommand.add_actuators()->set_position(BaseFeedback.actuators(i).position()); //BaseFeedback is updated in get_joint_angles() function
      if(servoing_mode == JOINT_POS_LOW_LEVEL){
        previous_setpoints[i] = initial_angles[i];
      }
    }
    gripper_initial_position = BaseFeedback.interconnect().gripper_feedback().motor()[0].position();
    actual_velocity_gripper = BaseFeedback.interconnect().gripper_feedback().motor()[0].velocity();
    actual_current_gripper = BaseFeedback.interconnect().gripper_feedback().motor()[0].current_motor();
    target_aperture = gripper_initial_position;
    actual_aperture = gripper_initial_position;
    BaseCommand.mutable_interconnect()->mutable_command_id()->set_identifier(0);

    GripperCommand = BaseCommand.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    GripperCommand->set_position(gripper_initial_position );
    GripperCommand->set_velocity(0.0);
    GripperCommand->set_force(force_gripper);

    auto fct_callback_noti = [&](k_api::Base::ActionNotification data)
    {
        int action_event_index = data.action_event();
        int action_type = data.handle().action_type();

        if(action_event_index == k_api::Base::ActionEvent::ACTION_END){
          if(action_type == k_api::Base::ActionType::SEND_GRIPPER_COMMAND && event_port.connected()){
              BaseFeedbackNotification = pBaseCyclicRT->RefreshFeedback();
              if(fabs(BaseFeedbackNotification.interconnect().gripper_feedback().motor()[0].position()/100-1)<0.01 || fabs(BaseFeedbackNotification.interconnect().gripper_feedback().motor()[0].position()/100)<0.01){
                event_port.write("gripper_done_no_obj");
              }
              else{
                event_port.write("gripper_done_with_obj");
              }
          }
          else if(action_type == k_api::Base::ActionType::REACH_POSE && event_port.connected()){
              event_port.write("cart_done");
          }
          else if(action_type == k_api::Base::ActionType::REACH_JOINT_ANGLES && event_port.connected()){
              event_port.write("joints_done");
          }
        }
        if(action_event_index == k_api::Base::ActionEvent::ACTION_ABORT){
          if(action_type == k_api::Base::ActionType::SEND_GRIPPER_COMMAND && event_port.connected()){
              event_port.write("gripper_aborted");
              log( Warning ) << "Driver: Gripper action Aborted. gripper_aborted event generated through event_port" << endlog();
          }
          else if(action_type == k_api::Base::ActionType::REACH_POSE && event_port.connected()){
              event_port.write("cart_aborted");
              log( Warning ) << "Driver: Reach cartesian action Aborted. cart_aborted event generated through event_port" << endlog();
          }
          else if(action_type == k_api::Base::ActionType::REACH_JOINT_ANGLES && event_port.connected()){
              event_port.write("joints_aborted");
              log( Warning ) << "Driver: Reach joints action Aborted. joints_aborted event generated through event_port" << endlog();
          }
        }
        if(action_event_index == 3){
          log( Warning ) << "Driver: High-level action paused" << endlog();
        }
    };
    // Subscribe to ActionNotifications
    auto notifHandle = pBase->OnNotificationActionTopic(fct_callback_noti, k_api::Common::NotificationOptions());

     return true;
  }
}

/**
 * This function is called by the Execution Engine.
 */
void kinova_gen3::updateHook() {
  if (debug_mode_conf){
    end_time_period = std::chrono::steady_clock::now();
    elapsed_time_period = end_time_period-start_time_period;
    periodicity.write((double)elapsed_time_period.count());
    start_time_period = end_time_period;
  }

  if(check_connection() && servoing_mode == HIGH_LEVEL && !sending_setpoints){
    try{
      pBaseCyclicRT->RefreshFeedback_callback(lambda_fct_callback);
    }
    catch(k_api::KDetailedException& ex){
        auto errorInfo = ex.getErrorInfo();
        auto errorCode = errorInfo.getError();
        log( Error ) << "Could not read sensor information. The component has stopped" << endlog();
        log( Error ) << "KDetailedException detected toStr: " << ex.toString().c_str() << endlog();
        log( Error ) << "KDetailedoption detected what:  " << ex.what() << endlog();

        log( Error ) << "KError error_code: " << errorCode.error_code() << endlog();
        log( Error ) << "KError sub_code: " << errorCode.error_sub_code() << endlog();
        log( Error ) << "KError sub_string: " << errorCode.error_sub_string() << endlog();
        this->stop();
        return;
    }
    catch (std::runtime_error& ex2)
    {
      log( Error ) << "Failed to read feedback from the robot. The component has stopped. " << endlog();
      log( Error ) << "Error: " << ex2.what() << endlog();
      this->stop();
      return;
    }
    catch(...){
      log( Error ) << "Could not read sensor information. The component has stopped" << endlog();
      this->stop();
      return;
    }
  }
  else if (servoing_mode == JOINT_POS_LOW_LEVEL && check_connection() && sending_setpoints){
    position_low_level_servoing();
  }
  else if (servoing_mode == JOINT_VEL_LOW_LEVEL && check_connection() && sending_setpoints){
    velocity_low_level_servoing();
  }
  if (begin_counting) {
    std::this_thread::sleep_until(end_time_sleep); //TODO: Delete this line, since it might be not necesary
    while ( std::chrono::steady_clock::now() < end_time_sleep || errno == EINTR ) { // In case the sleep was interrupted, continues to execute it
      errno = 0;
      std::this_thread::sleep_until(end_time_sleep);
    }
  }
  else{
    begin_counting = true;
  }
  end_time_sleep = std::chrono::steady_clock::now() + period_nano;

  if (!this->trigger()){
    log( Error ) << "The activity cannot be triggered" << endlog();
  }

}

/**
 * This function is called when the task is stopped.
 */
void kinova_gen3::stopHook() {
   set_servoing_mode(HIGH_LEVEL);
}


/**
 * This function is called when the update hook is on a break for some reason
 */
bool kinova_gen3::breakUpdateHook() {
   log( Error ) << "Entered to the breakUpdateHook" << endlog();
   return false;
}

/**
 * This function is called when the task is being deconfigured.
 */
void kinova_gen3::cleanupHook() {
   // Your configuration cleanup code
   kinova_gen3::stop_connection();
   std::cout << "The API was destroyed successfully!" << std::endl;

}


ORO_CREATE_COMPONENT(kinova_gen3)
