# LPC LABJACK T7 PYTHON CONFIG SCRIPT
# Author: Thomas Tedeschi
# Last Update Date: 4/19/2026

from labjack import ljm
from colorama import Fore, Style, init
import time
import numpy as np

mb = ljm.openS("T7", "ANY", "ANY")

# ================================
# FSM STATES
# ================================
STATE_ABORT           = 0
STATE_COLD_OPS        = 1
STATE_PRE_FIRE_PURGE  = 2
STATE_FILL            = 3
STATE_STATE_CHECK     = 4
STATE_HOT_FIRE        = 5
STATE_POST_FIRE_PURGE = 6
STATE_VENT_SAFING     = 7

# ================================
# UNIT CONSTANTS
# ================================
CELSIUS    = 1
FAHRENHEIT = 2

# ================================
# HARDWARE PIN ASSIGNMENTS
# ================================
# pwm_freq =
# tc1_pin_pos =
# tc1_pin_neg =
# tc2_pin_pos =
# tc2_pin_neg =
# tc_rng =
# tc_rind =
# tc_sett =
# pt1_pin =        # Fuel tank PT
# pt2_pin =        # Ox tank PT
# pt3_pin =        # Chamber PT
# pt4_pin =        # Supply PT
# pt_rng =
# pt_rind =
# pt_res_val =     # shunt resistor (ohms)
# lc_pin =
# lc_rng =
# lc_rind =
# v_off =          # load cell zero voltage
# kload =          # known calibration load (units)
# v_kload =        # voltage at kload
# v1_pin =         # Main Fuel valve     (two-way, powered=OPEN)
# v2_pin =         # Main Ox valve       (two-way, powered=OPEN)
# v3_pin =         # Fill valve          (two-way, powered=OPEN)
# v4_pin =         # Purge A             (two-way, powered=OPEN)
# v5_pin =         # Purge B             (two-way, powered=OPEN)
# v6_pin =         # Vent                (auto-return, unpowered=OPEN, powered=CLOSED)
# v1_open_pwm =
# v1_close_pwm =
# v2_open_pwm =
# v2_close_pwm =
# v3_open_pwm =
# v3_close_pwm =
# v4_open_pwm =
# v4_close_pwm =
# v5_open_pwm =
# v5_close_pwm =
# v6_close_pwm =   # V6 has no open_pwm — dropping EF enable is how it opens
# ignite_pin =
# kill_pin =       # digital output: normally HIGH; write LOW on abort to cut main 12V bus
# start_pin =      # digital input: rising edge starts sequence (STATE 1 → 2)
# arm_pin =        # digital input: rising edge in STATE 4 authorizes fire

# startval =       # initial PWM value written to all servo pins on boot

# ================================
# SEQUENCE TIMING
# ================================
# Purge_Time =              # seconds — pre/post fire purge duration
# Delay_1 =                 # seconds — igniter ON → V1 open
# Delay_2 =                 # seconds — V1 open → V2 open
# Burn_Duration =           # seconds — V2 open → close all + igniter off
# Ignition_Confirm_Time =   # seconds — window after V2 open to confirm ignition

# ================================
# PRESSURE PARAMETERS
# ================================
# pt1_pmin =   pt1_pmax =   # Fuel tank PT range (psi)
# pt2_pmin =   pt2_pmax =   # Ox tank PT range (psi)
# pt3_pmin =   pt3_pmax =   # Chamber PT range (psi)
# pt4_pmin =   pt4_pmax =   # Supply PT range (psi)
# MEOP =                    # Ox tank fill cutoff pressure (psi)
# Max_Chamber_Pressure =    # Hard start abort limit (psi)
# Min_Ignition_Pressure =   # Min chamber pressure after V2 open to confirm ignition

# ================================
# SAFETY THRESHOLDS
# ================================
# warn_temp_max =    warn_temp_min =
# fault_temp_max =   fault_temp_min =
# warn_pres_max =    warn_pres_min =
# fault_pres_max =   fault_pres_min =
# warn_load_max =    warn_load_min =
# fault_load_max =   fault_load_min =
# max_temp_rate =    # max per-sample delta before fault
# max_pres_rate =
# max_load_rate =
# amb =              # expected ambient temperature (C) for cold ops / vent safing checks
# tdiff =            # allowable TC deviation from ambient (C)
# pdiff =            # allowable PT deviation from 0 psig

REQUIRED_SAFE = 5
HIST          = 10

# ================================
# RUNTIME STATE
# ================================
system_state    = STATE_COLD_OPS
state_timer     = -1.0   # -1.0 = not yet entered current state
hot_fire_step   = 0      # sub-step within STATE_HOT_FIRE (0–3)
ignite          = False
fire_authorized = False
timestamp       = 0

pastt1 = np.zeros(HIST)
pastt2 = np.zeros(HIST)
pastp1 = np.zeros(HIST)
pastp2 = np.zeros(HIST)
pastp3 = np.zeros(HIST)
pastp4 = np.zeros(HIST)
pastl  = np.zeros(HIST)


# ================================
# EXCEPTIONS
# ================================
class SystemFault(Exception):
    pass


# ================================
# CONFIG FUNCTIONS
# ================================

def configure_thermocouple(ppin, npin, rng, rind, sett, temp_unit):
    pos_name = f"AIN{ppin}"
    ljm.eWriteName(mb, f"AIN{ppin}_NEGATIVE_CH", npin)
    ljm.eWriteName(mb, f"{pos_name}_RANGE", rng)
    ljm.eWriteName(mb, f"{pos_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(mb, f"{pos_name}_SETTLING_US", sett)
    ljm.eWriteName(mb, f"{pos_name}_EF_INDEX", 22)
    ljm.eWriteName(mb, f"{pos_name}_EF_CONFIG_A", temp_unit)

def configure_transducer_loadcell(apin, rng, rind):
    ain_name = f"AIN{apin}"
    ljm.eWriteName(mb, f"{ain_name}_NEGATIVE_CH", 199)
    ljm.eWriteName(mb, f"{ain_name}_RANGE", rng)
    ljm.eWriteName(mb, f"{ain_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(mb, f"{ain_name}_SETTLING_US", 0)

def configure_digital_io(diopin_num, func):
    if func == "input":
        ljm.eWriteName(mb, f"DIO{diopin_num}_DIRECTION", 0)
    else:
        ljm.eWriteName(mb, f"DIO{diopin_num}_DIRECTION", 1)
        ljm.eWriteName(mb, f"DIO{diopin_num}_STATE", 0)

def configure_clock(freq):
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ENABLE", 0)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_DIVISOR", 1)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ROLL_VALUE", 80000000 / freq)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ENABLE", 1)

def configure_pwm(diopin_num, val):
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_ENABLE", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_INDEX", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_OPTIONS", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_CONFIG_A", val)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_ENABLE", 1)


# ================================
# MEASUREMENT FUNCTIONS
# ================================

def measure_transducer_current(apin, resistance):
    return ljm.eReadName(mb, f"AIN{apin}") / resistance

def current_to_pressure(current, pmin, pmax):
    return ((current - 0.004) / 0.016) * (pmax - pmin) + pmin

def read_temperature(tpin):
    return ljm.eReadName(mb, f"AIN{tpin}_EF_READ_A")

def read_pressure(prpin, resis, p_min, p_max):
    return current_to_pressure(measure_transducer_current(prpin, resis), p_min, p_max)

def read_load(ldpin, v_off, kload, v_kload):
    factor = kload / (v_kload - v_off)
    return (ljm.eReadName(mb, f"AIN{ldpin}") - v_off) * factor

def dread(diopin_num):
    return ljm.eReadName(mb, f"DIO{diopin_num}")

def dwrite(diopin_num, state):
    ljm.eWriteName(mb, f"DIO{diopin_num}_STATE", state)

# ================================
# SAFETY FUNCTIONS
# ================================

def check_rate_of_change(history, current, max_delta):
    if timestamp == 0:
        return True
    last = history[(timestamp - 1) % HIST]
    return abs(current - last) <= max_delta

def check_sensor_avg(history, current, lo, hi):
    count = min(timestamp, HIST)
    avg = (np.sum(history[:count]) + current) / (count + 1)
    return lo <= avg <= hi

def run_safety_checks(tc1, tc2, pt1, pt2, pt3, pt4, load):
    temps   = [tc1, tc2]
    presses = [pt1, pt2, pt3, pt4]
    hists_t = [pastt1, pastt2]
    hists_p = [pastp1, pastp2, pastp3, pastp4]

    if system_state != STATE_HOT_FIRE
    for t, h in zip(temps, hists_t):
        if not check_rate_of_change(h, t, max_temp_rate):
            return False, "TEMPERATURE RATE OF CHANGE EXCEEDED"
        if not check_sensor_avg(h, t, fault_temp_min, fault_temp_max):
            return False, "TEMPERATURE OUT OF FAULT BOUNDS"
        if not (warn_temp_min <= t <= warn_temp_max):
            print(Fore.YELLOW + "WARNING: TEMPERATURE APPROACHING LIMITS")

    for p, h in zip(presses, hists_p):
        if not check_rate_of_change(h, p, max_pres_rate):
            return False, "PRESSURE RATE OF CHANGE EXCEEDED"
        if not check_sensor_avg(h, p, fault_pres_min, fault_pres_max):
            return False, "PRESSURE OUT OF FAULT BOUNDS"
        if not (warn_pres_min <= p <= warn_pres_max):
            print(Fore.YELLOW + "WARNING: PRESSURE APPROACHING LIMITS")

    if not check_rate_of_change(pastl, load, max_load_rate):
        return False, "LOAD RATE OF CHANGE EXCEEDED"
    if not check_sensor_avg(pastl, load, fault_load_min, fault_load_max):
        return False, "LOAD OUT OF FAULT BOUNDS"
    if not (warn_load_min <= load <= warn_load_max):
        print(Fore.YELLOW + "WARNING: LOAD APPROACHING LIMITS")

    return True, ""

def update_history(tc1, tc2, pt1, pt2, pt3, pt4, load):
    idx = timestamp % HIST
    pastt1[idx] = tc1
    pastt2[idx] = tc2
    pastp1[idx] = pt1
    pastp2[idx] = pt2
    pastp3[idx] = pt3
    pastp4[idx] = pt4
    pastl[idx]  = load

# --------------------------------
# ABORT PROTOCOL (global override)
# V1-V5 CLOSED. Igniter OFF. V6 released (springs OPEN to dump Ox).
# --------------------------------
def abort(reason=""):
    global system_state
    move(v1_pin, "closed", v1_open_pwm, v1_close_pwm)
    move(v2_pin, "closed", v2_open_pwm, v2_close_pwm)
    move(v3_pin, "closed", v3_open_pwm, v3_close_pwm)
    move(v4_pin, "closed", v4_open_pwm, v4_close_pwm)
    move(v5_pin, "closed", v5_open_pwm, v5_close_pwm)
    v6_release()
    fire_off()
    dwrite(kill_pin, 0)
    system_state = STATE_ABORT
    print(Fore.RED + ("ABORT" + (f": {reason}" if reason else "")))
    print(Fore.RED + "MANUAL RESET REQUIRED")

# ================================
# CONFIGURATION
# ================================
configure_thermocouple(tc1_pin_pos, tc1_pin_neg, tc_rng, tc_rind, tc_sett, CELSIUS)
configure_thermocouple(tc2_pin_pos, tc2_pin_neg, tc_rng, tc_rind, tc_sett, CELSIUS)
configure_transducer_loadcell(pt1_pin, pt_rng, pt_rind)
configure_transducer_loadcell(pt2_pin, pt_rng, pt_rind)
configure_transducer_loadcell(pt3_pin, pt_rng, pt_rind)
configure_transducer_loadcell(pt4_pin, pt_rng, pt_rind)
configure_transducer_loadcell(lc_pin, lc_rng, lc_rind)
configure_clock(pwm_freq)
for pin in [v1_pin, v2_pin, v3_pin, v4_pin, v5_pin, v6_pin]:
    configure_digital_io(pin, "output")
    configure_pwm(pin, startval)
configure_digital_io(ignite_pin, "output")
configure_digital_io(kill_pin,   "output")
ljm.eWriteName(mb, f"DIO{kill_pin}_STATE", 1)  # bus live at startup
configure_digital_io(arm_pin,    "input")
configure_digital_io(start_pin,  "input")

def sample_safety_check():
    tc1  = read_temperature(tc1_pin_pos)
    tc2  = read_temperature(tc2_pin_pos)
    pt1  = read_pressure(pt1_pin, pt_res_val, pt1_pmin, pt1_pmax)
    pt2  = read_pressure(pt2_pin, pt_res_val, pt2_pmin, pt2_pmax)
    pt3  = read_pressure(pt3_pin, pt_res_val, pt3_pmin, pt3_pmax)
    pt4  = read_pressure(pt4_pin, pt_res_val, pt4_pmin, pt4_pmax)
    load = read_load(lc_pin, v_off, kload, v_kload)

    # Global safety — runs every tick except in ABORT/VENT_SAFING where we wait for manual reset
    if system_state not in (STATE_ABORT, STATE_VENT_SAFING):
        safe, fault_reason = run_safety_checks(tc1, tc2, pt1, pt2, pt3, pt4, load)
        if not safe:
            abort(fault_reason)
    update_history(tc1, tc2, pt1, pt2, pt3, pt4, load)

    print(Fore.GREEN + f"TC1: {tc1:.1f}C  TC2: {tc2:.1f}C")
    print(Fore.GREEN + f"PT1: {pt1:.1f}  PT2: {pt2:.1f}  PT3: {pt3:.1f}  PT4: {pt4:.1f} psi")
    print(Fore.GREEN + f"Load: {load:.2f}  State: {system_state}")
    timestamp += 1
    return tc1, tc2, pt1, pt2, pt3, pt4, load, pastt1, pastt2, pastp1, pastp2, pastp3, pastp4, pastl