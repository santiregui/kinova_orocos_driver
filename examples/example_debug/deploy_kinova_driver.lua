require "rttlib"
require "rttros"
require "utils"

mode = "velocity"
motion_freq = 100
debug = true


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

dir = rtt.provides("ros"):find("kinova_driver") .. "/examples/example_debug/"

--Load the driver
depl:import("kinova_driver")
depl:loadComponent("kin", "kinova_gen3")
kin = depl:getPeer("kin")

kin:getProperty("debug_mode"):set(debug) --needs to be changed before kin:configure()
kin:configure()
cp=rtt.Variable("ConnPolicy")

--Load the component generating the trajectory
depl:loadComponent("traj_gen", "OCL::LuaComponent")
traj_gen = depl:getPeer("traj_gen")
if mode == "position" then
  traj_gen:exec_file(dir .. "low_level_position_component.lua")
  traj_gen:getProperty("initial_angles"):set(kin:get_joint_angles())
  depl:connect("kin.control_joint_positions","traj_gen.desired_positions",cp )
  port_name_setpoints = "desired_positions"
  print("Position mode!")
elseif mode == "velocity" then
  traj_gen:exec_file(dir .. "low_level_velocity_component.lua")
  depl:connect("kin.control_joint_velocities","traj_gen.desired_velocities",cp )
  port_name_setpoints = "desired_velocities"
  print("Velocity mode!")
end
depl:connect("kin.sensor_joint_angles","traj_gen.measured_angles",cp )
traj_gen:configure()

--Connect the ports

--Call configureHook() functions of both components
traj_gen:configure()

--Set the control mode to low level position servoing (2)
if mode == "position" then
  kin:set_servoing_mode(2)
elseif mode == "velocity" then
  kin:set_servoing_mode(1)
end



--Set the activity of both components running at 1 Khz (same frequency as the kinova robot control loop)
depl:setActivity("kin", 0, 99, rtt.globals.ORO_SCHED_RT)
depl:setActivity("traj_gen", 1/motion_freq, 10, rtt.globals.ORO_SCHED_RT)

-- depl:setWaitPeriodPolicy("kin", rtt.globals.ORO_WAIT_REL)
depl:setWaitPeriodPolicy("kin", rtt.globals.ORO_WAIT_ABS)
-- depl:setWaitPeriodPolicy("traj_gen", rtt.globals.ORO_WAIT_ABS)

kin:getProperty("setpoints_frequency"):set(motion_freq)



-- ====================================== Reporter =========================================
function exists(file)
  local ok, err, code = os.rename(file, file)
  if not ok then
if code == 13 then
  -- Permission denied, but it exists
  return true
end
  end
  return ok
end

function isdir(path)
  -- "/" works on both Unix and Windows
  return exists(path.."/")
end

date = os.date("%Y_%m_%d")
tmstamp = os.date("%Y_%m_%d_%H_%M_%S")
dir_name = "/reports_from_" .. date
dir_path = dir .. "/reports/all_data" .. dir_name

file_name = "/report_of_" .. tmstamp ..'.dat'

if not isdir(dir_path) then
  os.execute("mkdir -p " .. dir_path )
  print('Directory ' .. dir_name .. ' created')
end



--Start running the loop of both components.
kin:start()
traj_gen:start()

--This operation is used for telling the driver to start sending the setpoints to the robot
kin:start_sending_setpoints()

if debug then
  -- depl:loadComponent("Reporter","OCL::NetcdfReporting")
  depl:loadComponent("Reporter","OCL::FileReporting")
  reporter=depl:getPeer("Reporter")
  depl:connectPeers("kin","Reporter")
  depl:connectPeers("traj_gen","Reporter")
  reporter:reportPort("kin","periodicity")
  reporter:reportPort("kin","sensor_joint_angles")
  reporter:reportPort("kin","sensor_joint_velocities")
  reporter:reportPort("kin","sensor_joint_torques")
  reporter:reportPort("kin","sent_setpoints")
  reporter:reportPort("traj_gen",port_name_setpoints)
  depl:setActivity("Reporter", 1/motion_freq, 0, rtt.globals.ORO_SCHED_OTHER)


  reporter:getProperty("ReportFile"):set(dir_path .. file_name)
  reporter:configure()
  reporter:start()
end
