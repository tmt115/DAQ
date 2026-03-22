# LPC LABJACK T7 PYTHON CONFIG SCRIPT
# Author: Thomas Tedeschi
# Last Update Date: 3/21/2026

from labjack import ljm
from colorama import Fore, Style, init
import time

# Open LabJack
handle = ljm.openS("T7", "ANY", "ANY")

# GLOBAL CONSTANTS
# global celsius = 1
# global farenheit = 2
# global tpin_pos_num =
# global tpin_neg_num =
# global trng =
# global trind =
# global tset =
# global ppin_num =
# global prng =
# global prind =
# global lpin_num =
# global lrng =
# global lrind =
# global pmax =
# global pmin =
# global kload =
# global v_kload =
# global res_val =
# global arm_pin =
# global fire_pin =
# global ignite_out_pin =
# global servo1_pwm_pin =
# global servo2_pwm_pin =
# global curpos =
# global startval =
# global openval =
# global closedval =
# global lastfire =

# ================================
# CONFIG FUNCTIONS
# ================================

def configure_thermocouple(ppin, npin, rng, rind, sett, temp):
    pos_name = f"AIN{ppin}"
    neg_name = f"AIN{ppin}_NEGATIVE_CH"

    ljm.eWriteName(handle, neg_name, npin)
    ljm.eWriteName(handle, f"{pos_name}_RANGE", rng)
    ljm.eWriteName(handle, f"{pos_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(handle, f"{pos_name}_SETTLING_US", sett)
    ljm.eWriteName(handle, f"{pos_name}_EF_INDEX", 22)
    ljm.eWriteName(handle, f"{pos_name}_EF_CONFIG_A", temp)

def configure_transducer_loadcell(apin, rng, rind):
    ain_name = f"AIN{apin}"
    ljm.eWriteName(handle, f"{ain_name}_NEGATIVE_CH", 199)
    ljm.eWriteName(handle, f"{ain_name}_RANGE", rng)
    ljm.eWriteName(handle, f"{ain_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(handle, f"{ain_name}_SETTLING_US", 0)

def configure_digital_io(diopin_num, func):
    if func == "input":
        ljm.eWriteName(handle, f"DIO{diopin_num}_DIRECTION", 0)
    else:
        ljm.eWriteName(handle, f"DIO{diopin_num}_DIRECTION", 1)
        ljm.eWriteName(handle, f"DIO{diopin_num}_STATE", 0)

def configure_pwm(diopin_num, startval):
    ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 0)
    ljm.eWriteName(handle, "DIO_EF_CLOCK0_DIVISOR", 1)
    ljm.eWriteName(handle, "DIO_EF_CLOCK0_ROLL_VALUE", 1600000)
    ljm.eWriteName(handle, "DIO_EF_CLOCK0_ENABLE", 1)

    ljm.eWriteName(handle, f"DIO{diopin_num}_EF_ENABLE", 0)
    ljm.eWriteName(handle, f"DIO{diopin_num}_EF_INDEX", 0)
    ljm.eWriteName(handle, f"DIO{diopin_num}_EF_OPTIONS", 0)
    ljm.eWriteName(handle, f"DIO{diopin_num}_EF_CONFIG_A", startval)
    ljm.eWriteName(handle, f"DIO{diopin_num}_EF_ENABLE", 1)

# ================================
# MEASUREMENT FUNCTIONS
# ================================

def measure_transducer_current(apin, resistance):
    voltage = ljm.eReadName(handle, f"AIN{apin}")
    return voltage / resistance

def current_to_pressure(current, pmin, pmax):
    return ((current - 0.004) / 0.016) * (pmax - pmin) + pmin

def read_temperature(tpin):
    return ljm.eReadName(handle, f"AIN{tpin}_EF_READ_A")

def read_pressure(prpin, resis, p_min, p_max):
    c = measure_transducer_current(prpin, resis)
    return current_to_pressure(c, p_min, p_max)

def read_load(ldpin, v_off, kload, v_kload):
    factor = kload / (v_kload - v_off)
    pin_v = ljm.eReadName(handle, f"AIN{ldpin}")
    return (pin_v - v_off) * factor

# ================================
# DIGITAL / SERVO FUNCTIONS
# ================================

def move(diopin_num, nextpos):
    if nextpos == curpos:
        return
    if nextpos == "open":
        ljm.eWriteName(handle, f"DIO{diopin_num}_EF_CONFIG_A", openval)
    else:
        ljm.eWriteName(handle, f"DIO{diopin_num}_EF_CONFIG_A", closedval)
    curpos = nextpos


def dread(diopin_num):
    return ljm.eReadName(handle, f"DIO{diopin_num}")

def dwrite(diopin_num, state):
    ljm.eWriteName(handle, f"DIO{diopin_num}_STATE", state)

# ================================
# IGNITION FUNCTIONS
# ================================

def is_ignition_safe(a, p, t, l, mp, mt, ml):
    if not a:
        print(Fore.RED + "IGNITION FAILURE: SYSTEM NOT ARMED")
        return False
    if p > mp:
        print(Fore.RED + "IGNITION FAILURE: PRESSURE TOO LARGE")
        return False
    if t > mt:
        print(Fore.RED + "IGNITION FAILURE: TEMPERATURE TOO HIGH")
        return False
    if l > ml:
        print(Fore.RED + "IGNITION FAILURE: LOAD TOO HIGH")
        return False
    return True

def fire_control(ipin, ftime, a, p, t, l, mp, mt, ml):
    if is_ignition_safe(a, p, t, l, mp, mt, ml):
        if lastfire == 0:
            print(Fore.ORANGE + "IGNITION ACTIVE")
            fire(ipin, ftime)
            print(Fore.ORANGE + "IGNITION COMPLETE")
            return True
        else:
            print(Fore.RED + "IGNITION FAILED: NO CONSECUTIVE IGNITES")
            return False
    else:
        return False

def fire(ipin, ftime):
    dwrite(ipin, 1)
    time.sleep(ftime)
    dwrite(ipin, 0)

# ================================
# CONFIGURATION
# ================================

# Configure Temperature Sensing
configure_thermocouple(tpin_pos_num, tpin_neg_num, trng, trind, tset, celsius)
# Configure Pressure Sensing
configure_transducer_loadcell(ppin_num, prng, prind)
# Configure Load Sensing
configure_transducer_loadcell(lpin_num, lrng, lind)
# Configure the Servo Pins
configure_digital_io(servo1_pwm_pin, "output")
configure_pwm(servo1_pwm_pin, startval)
configure_digital_io(servo2_pwm_pin, "output")
configure_pwm(servo2_pwm_pin, startval)
# Configure ignition pins
configure_digital_io(ignite_in_pin, "input")
configure_digital_io(ignite_out_pin, "output")
configure_digital_io(fire_pin, "input")

# ================================
# MAIN LOOP 
# ================================

while True:
    temp = read_temperature(tpin_pos_num)
    pres = read_pressure(ppin_num, res_val, pmin, pmax)
    load = read_load(lpin_num, v_off, kload, v_kload)

    armed = (dread(arm_pin) == 1)
    firing = (dread(fire_pin) == 1)

    if firing:
        did_fire = fire_control(ignite_out_pin, fire_time, pres, temp, load, max_pres, max_temp, max_load)
        lastfire = int(did_fire)

    print(Fore.GREEN + "Temperature (C):", temp)
    print(Fore.GREEN + "Pressure (psi):", pres)
    print(Fore.GREEN + "Force:", load)

    time.sleep(0.1)