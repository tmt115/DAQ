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


# ================================
# EXCEPTIONS
# ================================
class SystemFault(Exception):
    pass



# ================================
# VALVE / SERVO FUNCTIONS
# ================================

def move(pin, target, open_pwm, close_pwm):
    if target == "open":
        ljm.eWriteName(mb, f"DIO{pin}_EF_CONFIG_A", open_pwm)
    elif target == "closed":
        ljm.eWriteName(mb, f"DIO{pin}_EF_CONFIG_A", close_pwm)
    else:
        print(Fore.RED + f"MOVE FAILED: INVALID TARGET '{target}'")

def v6_power_closed():
    ljm.eWriteName(mb, f"DIO{v6_pin}_EF_CONFIG_A", v6_close_pwm)

def v6_release():
    # Drop PWM — spring returns V6 to open (vent/dump)
    ljm.eWriteName(mb, f"DIO{v6_pin}_EF_ENABLE", 0)

def dread(diopin_num):
    return ljm.eReadName(mb, f"DIO{diopin_num}")

def dwrite(diopin_num, state):
    ljm.eWriteName(mb, f"DIO{diopin_num}_STATE", state)


# ================================
# IGNITION FUNCTIONS
# ================================

def fire_on():
    global ignite
    dwrite(ignite_pin, 1)
    ignite = True
    print(Fore.YELLOW + "IGNITION ACTIVE")

def fire_off():
    global ignite
    dwrite(ignite_pin, 0)
    ignite = False
    print(Fore.YELLOW + "IGNITION OFF")

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

def transition_to(new_state):
    global system_state, state_timer
    system_state = new_state
    state_timer  = -1.0


# ================================
# FSM STATE HANDLERS
# ================================

# STATE 1: COLD OPS
# All valves closed. V6 unpowered (open). Igniter off.
# Polls until all PTs ~0 and TCs ~ambient. start_pin rising edge advances to STATE 2.
def handle_cold_ops(tc1, tc2, pt1, pt2, pt3, pt4):
    pts_at_zero = all(abs(p) <= pdiff for p in [pt1, pt2, pt3, pt4])
    tcs_at_amb  = all(amb - tdiff <= t <= amb + tdiff for t in [tc1, tc2])
    if pts_at_zero and tcs_at_amb:
        if dread(start_pin) == 1:
            transition_to(STATE_PRE_FIRE_PURGE)
            print(Fore.CYAN + "STATE 2: PRE-FIRE PURGE")
    else:
        if not pts_at_zero:
            print(Fore.YELLOW + "COLD OPS: PTs NOT AT ZERO — WAITING")
        if not tcs_at_amb:
            print(Fore.YELLOW + "COLD OPS: TCs NOT AT AMBIENT — WAITING")

# STATE 2: PRE-FIRE PURGE
# V4 & V5 open. V1, V2, V3 closed. V6 unpowered (open). Igniter off.
# Runs for Purge_Time seconds, monitoring for anomalous back-pressure.
def handle_pre_fire_purge(pt1, pt2, pt3, pt4):
    global state_timer
    if state_timer < 0:
        move(v4_pin, "open", v4_open_pwm, v4_close_pwm)
        move(v5_pin, "open", v5_open_pwm, v5_close_pwm)
        state_timer = time.time()
        return
    for p in [pt1, pt2, pt3, pt4]:
        if p > warn_pres_max:
            abort("ANOMALOUS BACK-PRESSURE DURING PRE-FIRE PURGE")
            return
    if time.time() - state_timer >= Purge_Time:
        move(v4_pin, "closed", v4_open_pwm, v4_close_pwm)
        move(v5_pin, "closed", v5_open_pwm, v5_close_pwm)
        transition_to(STATE_FILL)
        print(Fore.CYAN + "STATE 3: FILL")

# STATE 3: FILL
# V6 powered closed FIRST, then V3 opens. V1, V2, V4, V5 closed.
# Auto-closes V3 when Ox tank PT (pt2) hits MEOP.
def handle_fill(pt2):
    global state_timer
    if state_timer < 0:
        v6_power_closed()
        move(v3_pin, "open", v3_open_pwm, v3_close_pwm)
        state_timer = time.time()
        return
    if pt2 >= MEOP:
        move(v3_pin, "closed", v3_open_pwm, v3_close_pwm)
        transition_to(STATE_STATE_CHECK)
        print(Fore.CYAN + "STATE 4: STATE CHECK — OX TANK AT MEOP")

# STATE 4: STATE CHECK (terminal count)
# V6 powered closed. All other valves closed. Igniter off.
# Verifies all sensors within GO range. Enables fire prompt on confirmation.
# Human gate: arm_pin rising edge authorizes hot fire.
def handle_state_check(tc1, tc2, pt1, pt2, pt3, pt4):
    global fire_authorized
    softsafe = (
        all(fault_temp_min <= t <= fault_temp_max for t in [tc1, tc2]) and
        all(fault_pres_min <= p <= fault_pres_max for p in [pt1, pt2, pt3, pt4])
    )
    if not softsafe:
        print(Fore.RED + "STATE CHECK: SENSORS NOT IN GO RANGE — HOLD")
        return
    if not fire_authorized:
        print(Fore.GREEN + "ALL SENSORS GO — AWAITING FIRE AUTHORIZATION (arm_pin)")
        fire_authorized = (dread(arm_pin) == 1)
    if fire_authorized:
        transition_to(STATE_HOT_FIRE)
        print(Fore.YELLOW + "STATE 5: HOT FIRE SEQUENCE INITIATED")

# STATE 5: HOT FIRE
# Fully automated timed sequence. V6 stays powered closed.
# V3, V4, V5 remain closed throughout.
# Chamber PT (pt3) monitored at max rate:
#   - Hard start: pt3 > Max_Chamber_Pressure → ABORT
#   - Ignition failure: pt3 flat after V2 open → ABORT
def handle_hot_fire(pt3):
    global hot_fire_step, state_timer
    if hot_fire_step == 0:
        fire_on()
        state_timer   = time.time()
        hot_fire_step = 1
    elif hot_fire_step == 1:
        if pt3 > Max_Chamber_Pressure:
            abort("HARD START — CHAMBER PRESSURE EXCEEDED LIMIT")
            hot_fire_step = 0
            return
        if time.time() - state_timer >= Delay_1:
            move(v1_pin, "open", v1_open_pwm, v1_close_pwm)
            state_timer   = time.time()
            hot_fire_step = 2
    elif hot_fire_step == 2:
        if pt3 > Max_Chamber_Pressure:
            abort("HARD START — CHAMBER PRESSURE EXCEEDED LIMIT")
            hot_fire_step = 0
            return
        if time.time() - state_timer >= Delay_2:
            move(v2_pin, "open", v2_open_pwm, v2_close_pwm)
            state_timer   = time.time()
            hot_fire_step = 3
    elif hot_fire_step == 3:
        if pt3 > Max_Chamber_Pressure:
            abort("HARD START — CHAMBER PRESSURE EXCEEDED LIMIT")
            hot_fire_step = 0
            return
        elapsed = time.time() - state_timer
        if elapsed >= Ignition_Confirm_Time and pt3 < Min_Ignition_Pressure:
            abort("IGNITION FAILURE — NO CHAMBER PRESSURE AFTER VALVE OPEN")
            hot_fire_step = 0
            return
        if elapsed >= Burn_Duration:
            move(v1_pin, "closed", v1_open_pwm, v1_close_pwm)
            move(v2_pin, "closed", v2_open_pwm, v2_close_pwm)
            fire_off()
            hot_fire_step = 0
            transition_to(STATE_POST_FIRE_PURGE)
            print(Fore.CYAN + "STATE 6: POST-FIRE PURGE")

# STATE 6: POST-FIRE PURGE
# V6 stays powered closed. V4 & V5 open. V1, V2, V3 closed.
# Waits until chamber PT (pt3) drops to ~0 before advancing.
def handle_post_fire_purge(pt3):
    global state_timer
    if state_timer < 0:
        move(v4_pin, "open", v4_open_pwm, v4_close_pwm)
        move(v5_pin, "open", v5_open_pwm, v5_close_pwm)
        state_timer = time.time()
        return
    if abs(pt3) <= pdiff:
        move(v4_pin, "closed", v4_open_pwm, v4_close_pwm)
        move(v5_pin, "closed", v5_open_pwm, v5_close_pwm)
        transition_to(STATE_VENT_SAFING)
        print(Fore.CYAN + "STATE 7: VENT & SAFING")

# STATE 7: VENT & SAFING
# Drop V6 power — spring swings OPEN to vent Ox tank. All other valves closed.
# Displays PAD SAFE only when all PTs read ~0 and TCs return to ambient.
def handle_vent_safing(tc1, tc2, pt1, pt2, pt3, pt4):
    global state_timer
    if state_timer < 0:
        v6_release()
        state_timer = time.time()
        return
    pts_safe = all(abs(p) <= pdiff for p in [pt1, pt2, pt3, pt4])
    tcs_safe = all(amb - tdiff <= t <= amb + tdiff for t in [tc1, tc2])
    if pts_safe and tcs_safe:
        print(Fore.GREEN + "PAD SAFE")


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


# ================================
# MAIN LOOP
# ================================
while True:
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

    if system_state == STATE_COLD_OPS:
        handle_cold_ops(tc1, tc2, pt1, pt2, pt3, pt4)
    elif system_state == STATE_PRE_FIRE_PURGE:
        handle_pre_fire_purge(pt1, pt2, pt3, pt4)
    elif system_state == STATE_FILL:
        handle_fill(pt2)
    elif system_state == STATE_STATE_CHECK:
        handle_state_check(tc1, tc2, pt1, pt2, pt3, pt4)
    elif system_state == STATE_HOT_FIRE:
        handle_hot_fire(pt3)
    elif system_state == STATE_POST_FIRE_PURGE:
        handle_post_fire_purge(pt3)
    elif system_state == STATE_VENT_SAFING:
        handle_vent_safing(tc1, tc2, pt1, pt2, pt3, pt4)
    elif system_state == STATE_ABORT:
        pass  # Waiting for manual reset and restart

    print(Fore.GREEN + f"TC1: {tc1:.1f}C  TC2: {tc2:.1f}C")
    print(Fore.GREEN + f"PT1: {pt1:.1f}  PT2: {pt2:.1f}  PT3: {pt3:.1f}  PT4: {pt4:.1f} psi")
    print(Fore.GREEN + f"Load: {load:.2f}  State: {system_state}")

    timestamp += 1
    time.sleep(0.001)