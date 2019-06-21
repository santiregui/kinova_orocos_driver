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
local joint_setpoints=rtt.Variable("array")

function configureHook()
    return true
end

function startHook()

 init_angle = initial_angles:get():totab()

    return true
end

time = 0
frequency = 0.3
amplitud = 20*math.pi/180 --Radians
function updateHook()

angle = math.sin(2*math.pi*frequency*time )*amplitud
mytab = {init_angle[1],init_angle[2],init_angle[3],init_angle[4],init_angle[5],init_angle[6],init_angle[7]+angle} --For positions

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
