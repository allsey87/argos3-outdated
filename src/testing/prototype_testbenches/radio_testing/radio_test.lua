-- Put your global variables here
variables = {};


--[[ This function is executed every time you press the 'execute'
     button ]]


function init()
   robot.leds.set_all_colors("blue");
   
   variables.id_byte = tonumber(string.sub(robot.id, 11));
	--data = {id_byte,22,39,24};
   --robot.radios.send("top", data);

	if(variables.id_byte == 2) then
		variables.STATE = "RED";
	else
		variables.STATE = "BLUE";
	end
end



--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	if(variables.STATE == "RED") then
      robot.leds.set_all_colors("red");
      variables.data = {variables.id_byte, 0};
      robot.radios.send("top", variables.data);
      variables.STATE = "BLUE";
	else
      robot.leds.set_all_colors("blue");
   end
	
   for k,v in pairs(robot.radios["top"]) do
      if (robot.radios["top"][k][0] ~= variables.id_byte) then
			logerr(robot.id .. ": received " .. tostring(robot.radios["top"][k][1]) .. " from " .. tostring(robot.radios["top"][k][0]))
			if	robot.radios["top"][k][1] == 0 then
   			   variables.STATE = "RED";
			end
	   else
			logerr(robot.id .. ": received message from self");

		end
   end
	

   --variables.data = {variables.id_byte, 255, 0}
   --robot.radios.send("top", variables.data);
 
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
