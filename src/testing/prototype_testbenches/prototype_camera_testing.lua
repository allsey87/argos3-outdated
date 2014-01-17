-- Put your global variables here



--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
   	if(robot.id == "robot") then
	   robot.prototype_colored_blob_forwards_camera.enable();
      robot.joints.set_joint_velocity("joint_1", "ANGULAR_X", 10);
      robot.joints.set_joint_velocity("joint_2", "ANGULAR_X", -10);
      robot.leds.set_all_colors("blue");
   end

	if(robot.id == "ledwall") then
		robot.leds.set_all_colors("red");
	end
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
   if(robot.id == "robot") then
     	log(robot.joints["joint_1"].rotation_x);
   end
end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   -- put your code here
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
