require "rttlib"
require "rfsm"
require "rfsm_rtt"
require "rfsmpp"




function loadrequire(module)
    local function requiref(module)
        require(module)
    end
    res = pcall(requiref,module)
    if not(res) then
        print('cannot find  ' .. module)
        return false
        else return true
    end
end



local tc=rtt.getTC();
local fsm
local fqn_out, events_in

state_machine_prop=rtt.Property("string","state_machine","state machine to execute")
tc:addProperty(state_machine_prop)
viz_on_prop=rtt.Property("bool","viz_on","save to file statemachine image")
tc:addProperty(viz_on_prop)
print_transition_prop=rtt.Property("bool","print_transition","print transitions")
tc:addProperty(print_transition_prop)

-- defalut values
viz_on_prop:set(false)
print_transition_prop:set(true)




function configureHook()
   -- load state machine
   fsm = rfsm.init(rfsm.load(state_machine_prop:get()) )

   -- enable state entry and exit dbg output
   if print_transition_prop:get() then
   		fsm.dbg=rfsmpp.gen_dbgcolor("fsm",
                   { STATE_ENTER=true, STATE_EXIT=true},
                   false)
 	 end
 	 ok=true
 	 if viz_on_prop:get() then
   		ok=ok and loadrequire("rfsm2uml")
			ok=ok and loadrequire("rfsm2tree")
			if not ok then
				rtt.logl('Warning', "cannot find/load modules 'rfsm2uml' or 'rfsm2tree'. Is graphiviz lua module installed? State machine will not be printed to dot and pdf files")
				viz_on_prop:set(false)
			end
 	 end


   -- redirect rFSM output to rtt log
   fsm.info=function(...) rtt.logl('Info', table.concat({...}, ' ')) end
   fsm.warn=function(...) rtt.logl('Warning', table.concat({...}, ' ')) end
   fsm.err=function(...) rtt.logl('Error', table.concat({...}, ' ')) end

   -- the following creates a string input port, adds it as a event
   -- driven port to the Taskcontext. The third line generates a
   -- getevents function which returns all data on the current port as
   -- events. This function is called by the rFSM core to check for
   -- new events.
   events_in = rtt.InputPort("string")
   tc:addEventPort(events_in, "events", "rFSM event input port")
   fsm.getevents = rfsm_rtt.gen_read_str_events(events_in)

   -- optional: create a string port to which the currently active
   -- state of the FSM will be written. gen_write_fqn generates a
   -- function suitable to be added to the rFSM step hook to do this.
   fqn_out = rtt.OutputPort("string")
   tc:addPort(fqn_out, "rFSM_cur_fqn", "current active rFSM state")
   rfsm.post_step_hook_add(fsm, rfsm_rtt.gen_write_fqn(fqn_out))
   return true
end

function updateHook()
 rfsm.run(fsm)
 if viz_on_prop:get() then
 	rfsm2uml.rfsm2dot(fsm, "fsm-uml.dot")
 	os.execute("dot fsm-uml.dot -Tpdf -O")
 end
end

function cleanupHook()
   -- cleanup the created ports.
   rttlib.tc_cleanup()
end
