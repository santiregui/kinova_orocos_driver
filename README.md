# <center> KINOVA DRIVER <center>

This repository contains the src code of an Orocos Component that can be used to control the Kinova Gen3 Ultra lightweight robot.

Before using it, it is recommended to read carefully the [User Guide](https://www.kinovarobotics.com/en/knowledge-hub/gen3-ultra-lightweight-robot) of the robot.

## I. Dependencies

To install and run this package, you need to have the following dependencies installed in computer with **Ubuntu 16.04** or **Ubuntu 18.04**

- OROCOS Toolchain version *2.9.0*
- gcc version *>=5.4.0*
- CMake version *>=3.10*

Other versions might as well work, but haven't been tested.

## II. Installation

To easily install the driver, ROS CATKIN is used. Installation with standard cmake/make is also possible but has not been tested (might be necessary to slightly modify the CMakeLists.txt files).

```shell
cd <Your installation directory>
git clone https://gitlab.mech.kuleuven.be/r0742557/kinova_driver.git
```

After cloning this repository, please download the [Kinova Kortex C++ API 2.2.0](https://artifactory.kinovaapps.com:443/artifactory/generic-public/kortex/API/2.2.0/linux_x86-64_x86_gcc.zip). Uncompress the file and copy the two folders that you find there (*include* and *lib*). Then paste them into the directory *kinova_driver/include/kortex_api/*

Then go to the root of your catkin workspace and run:

```shell
catkin_make
```
**Note:** The installation directory should be in the src folder of a catkin workspace



## III. How to use

In this guide we are going to use the LUA scripting language for deploying our Orocos application. For more information, please check out the [Orocos LUA Cookbook](http://www.orocos.org/wiki/orocos/toolchain/LuaCookbook) and the [API documentation](https://people.mech.kuleuven.be/~orocos/pub/devel/documentation/ocl/master/luaapi/).

#### General steps
**1.** Load the Orocos libraries and activate the printing in colors functionality (Looks prettier in the terminal!).
```lua
require "rttlib"
require "rttros"
require "utils"
rttlib.color=true
```

**2.** Load the driver component
```lua
tc=rtt.getTC()
depl=tc:getPeer("Deployer")
depl:import("kinova_driver")
depl:loadComponent("kin", "kinova_gen3")
kin = depl:getPeer("kin")
```

For observing the available ports, properties and operations (and a short documentation):
```lua
print(kin)
```

**3.** You can connect the ports to the port of another component in order to obtain sensor information. However, for this example, we are going to create a port in the deployer:

```lua
cp=rtt.Variable("ConnPolicy")
sensor_port = rtt.InputPort("array")
sensor_port:connect(kin:getPort("sensor_joint_angles"),cp)
fs, data =sensor_port:read()
print(data)
```
**4.** Setup a non-periodic activity for the driver. The internal code of the driver will make the periodicity of the component to be at 1kHz. This is because the control loop of the robot works at 1 kHz (consult the user guide for more information). Do not setup a periodicity manually or you will get an error.

```lua
depl:setActivity("kin", 0, 0, rtt.globals.ORO_SCHED_RT)
```

**5.** Setup the properties of the driver.
```
kin:getProperty("setpoints_frequency"):set(motion_freq)
kin:getProperty("ip_address"):set("192.168.1.10")
```
The first line sets the frequency in which you generate the setpoints. If the frequency is less than 1kHz, then the driver will linearly interpolate between the setpoints to keep a smooth motion at 1kHz. If the frequency is >= 1kHz, then the driver will not interpolate (however, it doesn't make sense to send setpoints at a higher frequency than 1kHz to the driver).
The second line is not necessary in case the ip_address is the default one (192.168.1.10).
**6.** Configure and start the execution of the driver

```lua
kin:configure() --Runs the configureHook() in the source code
kin:start() -- Runs the startHook() and starts the periodic execution of updateHook() (called by the Orocos execution engine)
```

For testing on the fly, you can print all values of the sensors in a JSON string format using the *get_all_sensor_jsonstring()* operation:

```lua
print(kin:get_all_sensor_jsonstring())
```
#### Driver servoing modes
<!-- #define HIGH_LEVEL 	0
#define JOINT_VEL_LOW_LEVEL 	1
#define JOINT_POS_LOW_LEVEL 	2 -->
There are three servoing modes in which you can configure the driver. This is setup with an operation as follows:

```lua
kin:set_servoing_mode(Value) --See the table for valid Values
```


Mode                      | Value                 | Interface
------------------------- | -------------------   | ----------------
HIGH_LEVEL                | <center> 0 <center>   | Orocos Operations
JOINT_VEL_LOW_LEVEL       |<center> 1 <center>    | Port: *control_joint_velocities*
JOINT_POS_LOW_LEVEL       | <center> 2 <center>   | Port: *control_joint_positions*


**HIGH_LEVEL**

This mode allows you to send a joint position or a cartesian pose command to the robot (once), and the robot will reach it (using the kinematic library of Kinova).

You can use the following operations when in HIGH_LEVEL mode:

- To change the aperture of the gripper (0 for fully opened, 1 for fully closed). Currently, the gripper supported is the ROBOTIQ 2F-85 adaptive gripper.
```lua
      kin:set_servoing_mode(0) -- Execute this once
      kin:change_gripper_aperture(0.5) --halfway opened
```
- To reach a position in the jointspace:
```lua
jvals_tab = {0,0,0,0,0,0,0} -- Sets all joints to 0 radians
jvals=rtt.Variable("array")
jvals:fromtab(jvals_tab)
kin:reach_joint_angles(jvals)
```

- To reach a position in the jointspace:
```lua
pose_tab = {x,y,z,roll,pitch,yaw} --Use your own values
pose=rtt.Variable("array")
pose:fromtab(pose_tab)
kin:reach_cartesian_pose(pose)
```


When the robot or the gripper reaches the desired position, you can get events (string notifications) through the event_port. This can be used when programming a finite state machine (as it is done in one of the examples using rFSM).

**JOINT_VEL_LOW_LEVEL**

This mode allows you to continuously send joint velocities at a frequency of 1 kHz (the control loop rate of the robot). You can also send the velocity setpoints at a lower rate, will linearly interpolate based on the setpoints_frequency you defined.

For using this mode you must first connect the port "*control_joint_velocities*" to the port of your trajectory generating component.
```lua
kin:set_servoing_mode(1) --Sets the servoing mode to low level velocity mode
kin:start_sending_setpoints() -- Starts sending the setpoints it gets from the port to the robot
```

Each time the set_servoing_mode() operation is called, it automatically stops sending the setpoints to the robot (for low level mode). Thus, start_sending_setpoints() should be called each time after you call set_servoing_mode() to set a low level mode.

If your trajectory generating components needs the initial joint angles (probably it would!) you can use the following before start sending the setpoints:

```lua
initial = kin:get_joint_angles() -- Obtain the joint angles in an array (rtt.Variable('array'))
```

**JOINT_POS_LOW_LEVEL**

This mode allows you to continuously send joint angles at a frequency of 1 kHz (the control loop rate of the robot). You can also send the velocity setpoints at a lower rate, will linearly interpolate based on the setpoints_frequency you defined  

For using this mode you must first connect the port "*control_joint_positions*" to the port of your trajectory generating component.
```lua
kin:set_servoing_mode(1) --Sets the servoing mode to low level velocity mode
kin:start_sending_setpoints() -- Starts sending the setpoints it gets from the port to the robot
```
Each time the set_servoing_mode() operation is called, it automatically stops sending the setpoints to the robot (for low level mode). Thus, start_sending_setpoints() should be called each time once after you call set_servoing_mode() to set a low level mode.

If your trajectory generating components needs the initial joint angles (probably it would!) you can use the following before start sending the setpoints:

```lua
initial = kin:get_joint_angles() -- Obtain the joint angles in an array (rtt.Variable('array'))
```
#### Ports

In all servoing modes you can access different sensor information though ports (the refresh rate of the port is the same as the frequency assigned to the kinova_driver activity).

Port                      |  Description
------------------------- |  -----------
sensor_joint_angles       |  7 elements with angles [rad]
sensor_joint_velocities   |  7 elements with velocities [rad/s]
sensor_joint_torques      |  7 elements with torques [Nm]
tool_pose                 |  3 elements with position [m] and 3 with RPY angles [rad]
tool_twist                |  3 elements with linear velocity [m/s] and 3 with angular velocity [rad/s]
tool_external_wrench*     |  3 elements with forces [N] and 3 with torques [Nm] (estimation by measuring actuator torques)
tool_imu                  |  3 IMU linear accelerations [m/s^2] and 3 IMU angular velocities [rad/s]
gripper_feedback        |  position [0-1], velocity [0-1] and current [mA] (first two are dimensionless)


In addition, there is a special port called "event_port", which generates string events when the robot reaches a position commanded while in high level servoing mode (except for gripper_done_no_obj and gripper_done_with_obj, which also work in low level mode).

Event                     | Description                           
------------------------- | ----------------------------------    
cart_done                 | Send when robot reaches a specified cartesian position (after *reach_cartesian_pose()*)   
joints_done               | Send when robot reaches a specified joint position (after *reach_joint_angles()*)    
gripper_done_no_obj       | Send when gripper reaches a specified aperture (after *change_gripper_aperture()* or *change_gripper_aperture_low_level()*)  
gripper_done_with_obj***    | Send when the gripper cannot reach the specified aperture because it grasped an object (after *change_gripper_aperture()* or *change_gripper_aperture_low_level()*)   
cart_aborted              | Send when a specified cartesian position is aborted****  
joints_aborted            | Send when a specified joint position is aborted****     
gripper_aborted           | Send when a specified aperture is aborted****   

**&ast;** Currently not supported by the Kortex API

**&ast;&ast;&ast;** It supports internal and external object grasping

**&ast;&ast;&ast;&ast;** One way it can be aborted is by pushung *RB* button in the Xbox controller

#### Properties
You can configure the following properties to change the default behaviour of the driver:

Property                     | Description                           
------------------------- | ----------------------------------    
ip_address                 | ip address of the kinova robot. It is by default set to the factory IP   
setpoints_frequency               | Frequency at which the low level setpoints are sent    
debug_mode       | If set to true before calling configureHook, additional ports are created to stream data related to timings  
velocity_limits    | Velocity limits of each joint, used for safety purposes [rad/s]. Set by default to the 'general limits' specified in the user manual  
acceleration_limits              | Acceleration limits of each joint, used for safety purposes [rad/s^2]. Set by default to the 'hard limits' specified in the user manual. Only checked when sending velocity setpoints (it's estimation from position is pretty bad).  
torque_limits            | Torque limits of each joint, used for safety purposes [Nm]. Set by default to the 'soft limits' specified in the user manual     
force_gripper           |Percentage of force applied by the gripper when grasping an object. Value from 0 to 100. Used when using change_gripper_aperture_low_level operation.   


## IV. Examples

In the directory *kinova_driver/examples* there are some examples (in different folders) that you can consult and use as a starting point for deploying your own applications.

- To run the **high level example**, type in the terminal:
```shell
cd kinova_driver/examples/example_high_level
rttlua -i deploy_kinova_driver.lua
```
Finite state machines can be implemented in rFSM by using the events obtained in the *event_port* of the driver. An event is sent throught the port after the robot completes the action commanded with the corresponding HIGH_LEVEL operation.

In this particular example, a simple pick and place task was programmed:

1. The robot opens the gripper
2. The robots goes to a position defined in the jointspace.
3. The robot keeps closing and opening the gripper until someone handles it an object (internal or external grasping).
4. The robot goes to a pose defined in the cartesian space.
5. The robot releases the object (e.g. if it is internal grasping it closes the gripper).
6. The robot returns to the same position defined in the jointspace and repeats

- To run the **low level position example**, type in the terminal:
```shell
cd kinova_driver/examples/example_low_level_position
rttlua -i deploy_kinova_driver.lua
```

- To run the **low level velocity example**, type in the terminal:
```shell
cd kinova_driver/examples/example_low_level_velocity
rttlua -i deploy_kinova_driver.lua
```
