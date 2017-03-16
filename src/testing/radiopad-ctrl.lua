function init()
end

function step()
	local rx = false;
	for radio,data in pairs(robot.radio_rx) do
		if (#data >= 1) then
			rx = true;
	      robot.leds.set_all_colors("red");
         robot.radio_tx.send("radio-x", {1});
      end
	end
	if(rx == false) then
		robot.leds.set_all_colors("black");
	end
end

function reset()
end

function destroy()
end
