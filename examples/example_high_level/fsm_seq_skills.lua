require("rtt")
require("rttlib")

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
kin    = depl:getPeer("kin")


return rfsm.state {
   configured = rfsm.state {
      entry=function()
        kin:configure()
        kin:set_servoing_mode(0)
        kin:start()
          end,
   },
--Used to open the gripper while in the position defined in the move_jointspace state
   joint_open_gripper = rfsm.state {
      entry=function()

              kin:set_servoing_mode(0)
              kin:change_gripper_aperture(0)

          end,
      exit=  function()

             end

   },
   --Used to open the gripper while in the position defined in the move cartesian states
   cart_open_gripper = rfsm.state {
      entry=function()
              kin:set_servoing_mode(0)
              kin:change_gripper_aperture(0)
          end,
      exit=  function()

             end

   },
      --Used to close the gripper while in the position defined in the move cartesian states
   cart_close_gripper = rfsm.state {
      entry=function()
          kin:set_servoing_mode(0)
          kin:change_gripper_aperture(1)
        end,
      exit=  function()

             end
   },
   --Used to close the gripper while in the position defined in move_jointspace
   joint_close_gripper = rfsm.state {
      entry=function()
          kin:set_servoing_mode(0)
          kin:change_gripper_aperture(1)
        end,
      exit=  function()

             end
   },

   --Used to move the robot to a desired position in the jointspace
   move_jointspace = rfsm.state {
      entry=function()
          kin:set_servoing_mode(0)
          val = 30;
          mytab = {val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180,val*math.pi/180}
          jvals=rtt.Variable("array")
          jvals:fromtab(mytab)
          kin:reach_joint_angles(jvals)
        end,
      exit=  function()

             end
   },

   --Used to move in cartesian space with the gripper closed.
   move_cartspace_closed = rfsm.state {
      entry=function()
          -- kin:set_servoing_mode(0)
          mytab = {0.8,0,0.36,0*math.pi/180,90*math.pi/180,0*math.pi/180} --Desired pose
          jvals=rtt.Variable("array")
          jvals:fromtab(mytab)
          kin:reach_cartesian_pose(jvals)
        end,
      exit=  function()

             end
   },
   --Used to move in cartesian space with the gripper opened.
   move_cartspace_opened = rfsm.state {
      entry=function()
          -- kin:set_servoing_mode(0)
          mytab = {0.8,0,0.36,0*math.pi/180,90*math.pi/180,0*math.pi/180} --Desired pose
          jvals=rtt.Variable("array")
          jvals:fromtab(mytab)
          kin:reach_cartesian_pose(jvals)
        end,
      exit=  function()

             end
   },

   rfsm.trans {src="initial", tgt="configured" },
   rfsm.trans {src="configured", tgt="joint_open_gripper", events={}},
   rfsm.trans {src="joint_open_gripper", tgt="move_jointspace", events={"gripper_done_no_obj"}},

   rfsm.trans {src="joint_close_gripper", tgt="move_cartspace_closed", events={"gripper_done_with_obj"}},
   rfsm.trans {src="joint_open_gripper", tgt="move_cartspace_opened", events={"gripper_done_with_obj"}},
   rfsm.trans {src="joint_close_gripper", tgt="joint_open_gripper", events={"gripper_done_no_obj"}},
   rfsm.trans {src="joint_open_gripper", tgt="joint_close_gripper", events={"gripper_done_no_obj"}},

   rfsm.trans {src="move_jointspace", tgt="joint_close_gripper", events={"joints_done"}},

   rfsm.trans {src="move_cartspace_closed", tgt="cart_open_gripper", events={"cart_done"}},
   rfsm.trans {src="move_cartspace_opened", tgt="cart_close_gripper", events={"cart_done"}},
   rfsm.trans {src="cart_open_gripper", tgt="move_jointspace", events={"gripper_done_no_obj"}},
   rfsm.trans {src="cart_open_gripper", tgt="move_jointspace", events={"gripper_done_with_obj"}},
   rfsm.trans {src="cart_close_gripper", tgt="move_jointspace", events={"gripper_done_no_obj"}},
   rfsm.trans {src="cart_close_gripper", tgt="move_jointspace", events={"gripper_done_with_obj"}},


}
