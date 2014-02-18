-- Put your global variables here



--[[ This function is executed every time you press the 'execute'
     button ]]
function init()
   	if(robot.id == "robot") then
	   --robot.prototype_forwards_camera.enable();
      --robot.leds.set_all_colors("blue");
      robot.joints.set_joint_velocity("joint_0", "ANGULAR_X", 5)
   end

	if(robot.id == "test-pattern") then
		robot.leds.set_single_color(1, "blue");
		robot.leds.set_single_color(2, "blue");
		robot.leds.set_single_color(3, "blue");
		robot.leds.set_single_color(4, "blue");
		robot.leds.set_single_color(5, "blue");
		robot.leds.set_single_color(6, "blue");
		robot.leds.set_single_color(7, "blue");
		robot.leds.set_single_color(8, "blue");
	end
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()

end



--[[ This function is executed every time you press the 'reset'
     button in the GUI. It is supposed to restore the state
     of the controller to whatever it was right after init() was
     called. The state of sensors and actuators is reset
     automatically by ARGoS. ]]
function reset()
   init();
end



--[[ This function is executed only once, when the robot is removed
     from the simulation ]]
function destroy()
   -- put your code here
end
