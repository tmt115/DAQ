--[[
    LPC LABJACK T7 LUA CONFIG SCRIPT
    Author: Thomas Tedeschi
    Last Updated Date: 3/21/2026
--]]

local mb = require("Modbus")

-- DEFINE LOCAL VARIABLES HERE --
--local celsius = 1
--local farenheit = 2
--local tpin_pos_num
--local tpin_neg_num
--local trng
--local trind
--local tset
--local ppin_num
--local prng
--local prind
--local lpin_num
--local lrng
--local lrind
--local pmax
--local pmin
--local kload
--local v_kload
--local res_val
--local ignite_in_pin
--local ignite_out_pin
--local servo1_pwm_pin
--local servo2_pwm_pin
--local curpos
--local startval
--local openval
--local closedval
---------------------------------

 --[[ 
    Configure one thermocouple with positive voltage ain at ppin against
    npin, resolution rind, voltage range +-0.1V, settling time of set us, 
    and temperature celsius or farenheit.
 --]]
function configure_thermocouple(ppin, npin, rng, rind, set, temp)
    local pos_name = "AIN" .. ppin
    local neg_name = "AIN" .. ppin .. "_NEGATIVE_CH"
    mb.writeName(neg_name, npin)
    mb.writeName(pos_name .. "_RANGE", rng)
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
function configure_transducer_loadcell(apin, rng, rind)
    local ain_name = "AIN" .. apin
    mb.writeName(ain_name .. "_NEGATIVE_CH", 199)
    mb.writeName(ain_name .. "_RANGE", rng)
    mb.writeName(ain_name .. "_RESOLUTION_INDEX", rind)
    mb.writeName(ain_name .. "_SETTLING_US", 0)
end

--[[
    Configures a DIO pin as an input or output using its pin number. 
    func MUST BE a string that is either 'input' or 'output'.
    Outputs will start at 0V.
--]]
function configure_digital_io(diopin_num, func)
    if func == "input" then
        mb.writeName("DIO" .. diopin_num .. "_DIRECTION", 0)
    else
        mb.writeName("DIO" .. diopin_num .. "_DIRECTION", 1)
        mb.writeName("DIO" .. diopin_num .. "_STATE", 0)
    end
end

--[[
    Configures a DIO pin for pwm output at 50Hz and the 
    given start value.
--]]
function configure_pwm(diopin_num, startval)
    mb.writeName("DIO_EF_CLOCK0_ENABLE", 0)
    mb.writeName("DIO_EF_CLOCK0_DIVISOR", 1)
    mb.writeName("DIO_EF_CLOCK0_ROLL_VALUE", 1600000)
    mb.writeName("DIO_EF_CLOCK0_ENABLE", 1)

    mb.writeName("DIO" .. diopin_num .. "_EF_ENABLE", 0)
    mb.writeName("DIO" .. diopin_num .. "_EF_INDEX", 0)
    mb.writeName("DIO" .. diopin_num .. "_EF_OPTIONS", 0)
    mb.writeName("DIO" .. diopin_num .. "_EF_CONFIG_A", startval)
    mb.writeName("DIO" .. diopin_num .. "_EF_ENABLE", 1)
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

--[[
    Uses positive AIN pin number for thermocouple to read the temperature
    at the given time in the configured units.
--]]
function read_temperature(tpin)
    return mb.readName("AIN" .. tpin .. "_EF_READ_A")
end

--[[
    Uses the known resistor value as well as the transducer AIN pin
    to find the pressure by finding the current and converting it
    into pressure.
--]]
function read_pressure(prpin, resis)
    local c = measure_transducer_current(pr_apin, resis)
    return current_to_pressure(c, p_min, p_max)
end

--[[
    Uses the AIN pin for the load cell, the offset or zero load 
    voltage, a known load and the voltage at that load to compute
    the load given the voltage.
--]]
function read_load(ldpin, v_off, kload, v_load)
    local factor = kload/(v_kload-v_off)
    local pin_v = mb.readName("AIN" .. ldpin)
    return (pin_v - v_off)*factor
end

--[[
    Simple two position servo movement function that will send the 
    motor connected to the given dio pin to the open or closed position 
    based on the command given. nextpos MUST BE 'open' or 'closed'.
--]]
function move(diopin_num, nextpos)
    if nextpos == curpos then
        return
    else if nextpos = "open" then
        mb.writeName("DIO" .. diopin_num .. "_EF_CONFIG_A", openval)
    else
        mb.writeName("DIO" .. diopin_num .. "_EF_CONFIG_A", closedval)
    end
end

-- Configure Temperature Sensing
configure_thermocouple(tpin_pos_num, tpin_neg_num, trng, trind, tset, celsius)
-- Configure Pressure Sensing
configure_transducer_loadcell(ppin_num, prng, prind)
-- Configure Load Sensing
configure_transducer_loadcell(lpin_num, lrng, lind)
-- Configure the Servo Pins
configure_digital_io(servo1_pwm_pin, "output")
configure_pwm(servo1_pwm_pin, startval)
configure_digital_io(servo2_pwm_pin, "output")
configure_pwm(servo2_pwm_pin, startval)
-- Configure ignition pins
configure_digital_io(ignite_in_pin, "input")
configure_digital_io(ignite_out_pin, "output")


-- Run Loop (IN CONSTRUCTION)
while(true)
    local temp = read_temperature(tpin_pos_num)
    local pres = read_pressure(ppin_num)
    local load = read_load(lpin_num)
    -- This is assuming Celsius can change the units
    print("Temperature (C): ", temp)
    print("Pressure (psi): ", pres)
    -- Not sure on units here
    print("Force (N): ", load)
end
