require("rttlib")
require("math")

--This component generates a sine wave for a joint position, beginning from the initial value of the corresponding joint

tc=rtt.getTC()

initial_angles=rtt.Property("array", "initial_angles", "The initial joint angles of the robots")
tc:addProperty(initial_angles)

measured_angles = rtt.InputPort("array", "measured_angles")
tc:addPort(measured_angles)

desired_positions = rtt.OutputPort("array", "desired_positions")
tc:addPort(desired_positions)

iterator = 0
init_angle = {}
angle = {0,0,0,0,0,0,0}
mytab = {0,0,0,0,0,0,0}
local joint_setpoints=rtt.Variable("array")

function configureHook()
    return true
end

function startHook()

 init_angle = initial_angles:get():totab()
 mytab = initial_angles:get():totab()
 angle = initial_angles:get():totab()

 -- joint_setpoints:fromtab(init_angle)
 -- desired_positions:write(joint_setpoints)
    return true
end

time = 0

frequency = {0.2,0.2,0.2,0.2,0.2,0.2,0.2}
amplitude = {30,20,20,20,20,20,30} --Degrees
tau = 10

function updateHook()

for n=1,7 do
  if n == 7 then
    angle[n] = init_angle[n] + (1-math.exp(-time/tau))*(amplitude[n]*math.pi/180)*(math.sin(2*math.pi*frequency[n]*time ))
  end
end
mytab = {angle[1],angle[2],angle[3],angle[4],angle[5],angle[6],angle[7]}

joint_setpoints:fromtab(mytab)
desired_positions:write(joint_setpoints)
time = time + tc:getPeriod()

    --Uncomment for printing the measured angles in the terminal:
    --local fs,val= measured_angles:read()
    -- myTable = val:totab()
    -- for k,v in pairs(myTable) do
    --   print("actuator  #" .. k .. "    value:  " .. round(v*180/math.pi,1) )
    -- end

end


function cleanupHook()

end

function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end
