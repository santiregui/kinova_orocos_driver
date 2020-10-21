require "rttlib"
require "rttros"
require "utils"


rttlib.color=true

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
dir = rtt.provides("ros"):find("kinova_driver") .. "/examples/example_high_level/"

depl:import("kinova_driver")
depl:loadComponent("kin", "kinova_gen3")
kin = depl:getPeer("kin")

depl:loadComponent("print_sensor", "OCL::LuaComponent")
print_sensor = depl:getPeer("print_sensor")
print_sensor:exec_file(dir .. "print_component.lua")
print_sensor:configure()



depl:loadComponent("Supervisor", "OCL::LuaComponent")
sup = depl:getPeer("Supervisor")
sup:exec_file(dir.."fsm_component.lua")
sup:getProperty("state_machine"):set(dir.."fsm_seq_skills.lua")


sup:addPeer(depl)
sup:configure()
sup:start()

cp=rtt.Variable("ConnPolicy")
depl:connect("kin.sensor_joint_angles","print_sensor.inport",cp )
depl:connect("kin.event_port","Supervisor.events",cp)

depl:setActivity("kin", 0, 0, rtt.globals.ORO_SCHED_OTHER)
depl:setActivity("print_sensor", 1, 0, rtt.globals.ORO_SCHED_OTHER) --Prints every second

--Uncomment to print sensor data:
-- print_sensor:start()




--Execute this function in the lua terminal in order to move all joints to the same angle (in degrees)
function test_joint(val)
  kin:set_servoing_mode(0)
  mytab = {val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180}
  -- mytab = {0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,0*math.pi/180,val*math.pi/180} --Uncoment to send just to one
  -- mytab[1] = val
  local jvals=rtt.Variable("array")
  jvals:fromtab(mytab)
  kin:reach_joint_angles(jvals)
end

--Execute this function in the lua terminal in order to change the y coordinate of the end effector.
function test_cart(val)
  kin:set_servoing_mode(0)
  mytab = {0.8,val,0.4,0,90*math.pi/180,0}
  -- mytab[1] = val
  local jvals=rtt.Variable("array")
  jvals:fromtab(mytab)
  kin:reach_cartesian_pose(jvals)
end
