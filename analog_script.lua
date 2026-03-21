--[[
    LPC LABJACK T7 LUA CONFIG SCRIPT
    Author: Thomas Tedeschi
    Last Updated Date: 3/21/2026
--]]

local mb = require("Modbus")

-- DEFINE LOCAL VARIABLES HERE --
--local celsius = 1
--local farenheit = 2
-- ...
---------------------------------

 --[[ 
    Configure one thermocouple with positive voltage ain at ppin against
    npin, resolution rind, voltage range +-0.1V, settling time of set us, 
    and temperature celsius or farenheit.
 --]]
function configure_thermocouple(ppin, npin, rind, set, temp)
    local pos_name = "AIN" .. ppin
    local neg_name = "AIN" .. ppin .. "_NEGATIVE_CH"
    mb.writeName(neg_name, npin)
    mb.writeName(pos_name .. "_RANGE", 0.1)
    mb.writeName(pos_name .. "_RESOLUTION_INDEX", rind)
    mb.writeName(pos_name .. "_SETTLING_US", set)
    mb.writeName(pos_name .. "_EF_INDEX", 22)
    mb.writeName(pos_name .. "_EF_CONFIG_A", temp)
end

--[[ 
    Configure one pressure transducer between input ain pin
    apin and ground, with voltage range +-range V, and resolution
    rind. 
    ***TRANSDUCER SHOULD GO THROUGH A RESISTOR TO TURN CURRENT TO 
    VOLTAGE.
--]]
function configure_transducer(apin, range, rind)
    local ain_name = "AIN" .. apin
    mb.writeName(ain_name .. "_NEGATIVE_CH", 199)
    mb.writeName(ain_name .. "_RANGE", range)
    mb.writeName(ain_name .. "_RESOLUTION_INDEX", rind)
    mb.writeName(ain_name .. "_SETTLING_US", 0)
end

--[[
    Uses the ain pin to measure the input voltage through
    a known resistor and calculates the current
--]]
function measure_transducer_current(apin, resistance)
    local voltage = mb.readName("AIN" .. apin)
    local curr = voltage/resistance
    return curr
end

--[[
    Takes in current to the ain to calculate the pressure using a formula
    based the min and max currents, which we know to be 4 and 20ma, and 
    the min and max pressures, which we can change.
--]]
function current_to_pressure(current, pmin, pmax)
    local pressure = ((current-0.004)/(0.016)) * (pmax-pmin) + pmin
    return pressure
end

-- Configure the pins
configure_thermocouple(pos_pin, neg_pin, temp_res_ind, settle, celsius)
configure_transducer(ain_pin, v_range, pres_res_ind)

-- Loop to keep checking our values and print them out
while(true)
    local temp = mb.readName(pos_name .. "_EF_READ_A")
    local c = measure_transducer_current(pr_apin, resistor)
    local p = current_to_pressure(c, p_min, p_max)

    -- This is assuming Celsius can change the units
    print("Temperature (C): ", temp)
    print("Pressure (psi): ", p)
end
