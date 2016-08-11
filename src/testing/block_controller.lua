local lookup_table = {
	["0"] = {r = 0, g = 0, b = 0},
	["1"] = {r = 255, g = 120, b = 255},
	["2"] = {r = 255, g = 208, b = 27},
	["3"] = {r = 74, g = 255, b = 27},
	["4"] = {r = 74, g = 255, b = 255},
}

function init()
   reset();  
end

--[[ This function is executed at each time step
     It must contain the logic of your controller ]]
function step()
	for radio,data in pairs(robot.radio_rx) do
		if (#data >= 1) then
			local setting = lookup_table[string.char(data[1][1])];
	      robot.leds.set_all_colors(setting.r,setting.g,setting.b);
		end
	end
end

function reset()
   local setting = lookup_table["0"];
   robot.leds.set_all_colors(setting.r,setting.g,setting.b);
end

function destroy()
   -- put your code here
end
