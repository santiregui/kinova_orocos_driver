require("rttlib")
require("math")

tc=rtt.getTC()


inport = rtt.InputPort("array", "inport")
tc:addPort(inport)


-- The Lua component starts its life in PreOperational, so
-- configureHook can be used to set stuff up.
function configureHook()

    return true
end

function startHook()
    return true
end

function updateHook()

local fs,val= inport:read()
    myTable = val:totab()

    for k,v in pairs(myTable) do
      print("actuator  #" .. k .. "    value:  " .. round(v*180/math.pi,1) .. "    degrees" )
    end

end


function cleanupHook()

end

function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end
