# <center> KINOVA DRIVER <center>

This repository contains the src code of an Orocos Component that can be used to control the Kinova Gen3 Ultra lightweight robot.

Before using it, it is recommended to read carefully the [User Guide](https://www.kinovarobotics.com/en/knowledge-hub/gen3-ultra-lightweight-robot) of the robot.

## I. Dependencies

To install and run this package, you need to have the following dependencies installed in computer with **Ubuntu 16.04**

- OROCOS Toolchain version *2.9.0*
- gcc version *5.4.0*
- CMake version *>=3.10*

Other versions might as well work, but haven't been tested.

## II. Installation

To easily install the driver, ROS CATKIN is used. Installation with standard cmake/make is also possible but has not been tested (might be necessary to slightly modify the CMakeLists.txt files).

```shell
cd <Your installation directory>
git clone https://gitlab.mech.kuleuven.be/r0742557/kinova_driver.git
```

After cloning this repository, please download the [Kinova Kortex C++ API](https://github.com/Kinovarobotics/kortex) (in there they provide a google drive download link). In the downloaded files, go to the directory `kortex_api-1.1.6/cpp/linux_gcc_x86-64` and copy the two folders that you find there (*include* and *lib*). Then paste them into the directory of this driver, so that they follow the following directory hierarchy:

```sh
kinova_driver/include/kortex_api/
┬  
├ include/
├ lib/  
└   ┬  
    ├ debug/  
    └ release/
```
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
**4.** Setup a periodic activity for the driver. If you are going to use the low level servoing mode, you should set it to 1 kHz (0.001 seconds). This is because the control loop of the robot works at 1 kHz (consult the user guide for more information). If you set it to a lower value, you will not obtain a smooth movement (it will also be very loudy and bad for the robot!).

```lua
depl:setActivity("kin", 0.001, 0, rtt.globals.ORO_SCHED_OTHER)
```
**5.** Configure and start the execution of the driver

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

This mode allows you to continuously send joint velocities at a frequency of 1 kHz (the control loop rate of the robot). You can also send the velocity setpoints at a lower rate, and the Kinova Driver will continue sending the old velocity values until a new one is found through the port.

For using this mode you must first connect the port "*control_joint_velocities*" to the port of your trajectory generating component.
```lua
kin:set_servoing_mode(1) --Sets the servoing mode to low level velocity mode
kin:start_sending_setpoints() -- Starts sending the setpoints it gets from the port to the robot
```

Each time the set_servoing_mode() operation is called, it automatically stops sending the setpoints to the robot (for low level mode). Thus, start_sending_setpoints() should be called each time after you call set_servoing_mode() to set a low level mode.

**Note:** The current version of the Kortex API (v1.1.6) doesn't support sending joint velocities. For this reason, in the meantime, the driver integrates the velocities and sends position setpoints to the robot. When v2.0 is released, the velocities will be sent directly to the robot.

If your trajectory generating components needs the initial joint angles (probably it would!) you can use the following before start sending the setpoints:

```lua
initial = kin:get_joint_angles() -- Obtain the joint angles in an array (rtt.Variable('array'))
```

**JOINT_POS_LOW_LEVEL**

This mode allows you to continuously send joint angles at a frequency of 1 kHz (the control loop rate of the robot). It is highly recommended to send the position setpoints at 1 kHz (in contrast to the velocity mode), or else you will obtain a very loudy and non-smooth motion (definitely not good for the robot!). Because the control loop of the robot works at 1 kHz, if you send positions at a lower frequency it would be equivalent to send a "staircase" function, in which the dwell sections are determined by the old values received by the port.  

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
gripper_feedback**        |  position [0-1], velocity [0-1] and force [0-1] (All dimensionless)


In addition, there is a special port called "event_port", which generates string events when the robot reaches a position commanded while in high level servoing mode.

Event                     | Description                           
------------------------- | ----------------------------------    
cart_done                 | Send when robot reaches a specified cartesian position (after *reach_cartesian_pose()*)   
joints_done               | Send when robot reaches a specified joint position (after *reach_joint_angles()*)    
gripper_done_no_obj       | Send when gripper reaches a specified aperture (after *change_gripper_aperture()*)  
gripper_done_with_obj***    | Send when the gripper cannot reach the specified aperture because it grasped an object (after *change_gripper_aperture()*)   
cart_aborted              | Send when a specified cartesian position is aborted****  
joints_aborted            | Send when a specified joint position is aborted****     
gripper_aborted           | Send when a specified aperture is aborted****   

**&ast;** Currently not supported by the Kortex API

**&ast;&ast;** Currently only position is supported by the Kortex API. You will get zeros in the last two elements of the array.

**&ast;&ast;&ast;** It supports internal and external object grasping

**&ast;&ast;&ast;&ast;** One way it can be aborted is by pushung *RB* button in the Xbox controller

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
