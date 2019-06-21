require "rttlib"
require "rttros"
require "utils"


rttlib.color=true

--If you set it to Info you will get more stuff printed from the driver
rtt.setLogLevel("Warning")

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end
depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("rtt_rospack")

dir = rtt.provides("ros"):find("kinova_driver") .. "/examples/example_low_level_position/"

depl:import("kinova_driver")

--Load the component generating the trajectory
depl:loadComponent("traj_gen", "OCL::LuaComponent")
traj_gen = depl:getPeer("traj_gen")
traj_gen:exec_file(dir .. "low_level_position_component.lua")
traj_gen:configure()

--Load the driver
depl:loadComponent("kin", "kinova_gen3")
kin = depl:getPeer("kin")



--Call configureHook() functions of both components
traj_gen:getProperty("initial_angles"):set(kin:get_joint_angles())

kin:configure()
kin:set_servoing_mode(2)

--Set the activity of both components running at 1 Khz (same frequency as the kinova robot control loop)
depl:setActivity("kin", 0.001, 0, rtt.globals.ORO_SCHED_OTHER) --Must run at 1 khz (0.001 ms), or else you will not get a smooth motion
depl:setActivity("traj_gen", 0.001, 0, rtt.globals.ORO_SCHED_OTHER)

--Connect the ports
cp=rtt.Variable("ConnPolicy")
depl:connect("kin.sensor_joint_angles","traj_gen.measured_angles",cp )
depl:connect("kin.control_joint_positions","traj_gen.desired_positions",cp )

traj_gen:configure()


--Set the control mode to low level position servoing (2)




--Start running the loop of both components.
kin:start()
traj_gen:start()

--This operation is used for telling the driver to start sending the setpoints to the robot
kin:start_sending_setpoints()