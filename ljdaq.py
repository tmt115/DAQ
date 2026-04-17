# LPC LABJACK T7 PYTHON CONFIG SCRIPT
# Author: Thomas Tedeschi
# Last Update Date: 3/22/2026

from labjack import ljm
from colorama import Fore, Style, init
import time
import numpy as np

# Open LabJack
mb = ljm.openS("T7", "ANY", "ANY")

# GLOBAL CONSTANTS
killed = False
timestamp = 0
celsius = 1
farenheit = 2
hist = 10
pastt = np.zeros(hist)
pastp = np.zeros(hist)
pastl = np.zeros(hist)
last_mc = 0
SERVO_IDLE   = 0
SERVO_STAGE1 = 1  
SERVO_STAGE2 = 2 
servo_state  = SERVO_IDLE
servo_timer  = 0
# pwm_freq =
# kill_pin_num =
# tpin_pos_num =
# tpin_neg_num =
# trng =
# trind =
# tset =
# ppin_num =
# prng =
# prind =
# lpin_num =
# lrng =
# lrind =
# pmax =
# pmin =
# kload =
# v_kload =
# res_val =
# arm_pin =
# safety_arm_pin =
# fire_pin =
# ignite_out_pin =
# servo1_pwm_pin =
# servo2_pwm_pin =
# curpos =
# startval =
# openval =
# movact_pin_num
# closedval =
# lastfire =
# v_off
# fire_time

# Safety thresholds — warning band triggers a log message, fault band triggers a kill
# warn_temp_max =
# warn_temp_min =
# fault_temp_max =
# fault_temp_min =
# warn_pres_max =
# warn_pres_min =
# fault_pres_max =
# fault_pres_min =
# warn_load_max =
# warn_load_min =
# fault_load_max =
# fault_load_min =

# Max allowed per-sample change before triggering a fault
# max_temp_rate =
# max_pres_rate =
# max_load_rate =

REQUIRED_SAFE = 5  # consecutive clean readings before clearing a fault

# ================================
# EXCEPTIONS
# ================================

# --------------------------------------------------------
# Abort sequence error to crash the program and force a 
# manual restart.
# --------------------------------------------------------
class SystemFault(Exception):
    pass

# ================================
# CONFIG FUNCTIONS
# ================================

# --------------------------------------------------------
# Configures a thermocoupler to read differential voltage between
# ppin and npin. Rng is the max voltage this can read, rind is 
# the resolution of the measurement, sett is settling time
# and temp is the unit (e.g. celsius or farenheit)
# --------------------------------------------------------

def configure_thermocouple(ppin, npin, rng, rind, sett, temp):
    pos_name = f"AIN{ppin}"
    neg_name = f"AIN{ppin}_NEGATIVE_CH"

    ljm.eWriteName(mb, neg_name, npin)
    ljm.eWriteName(mb, f"{pos_name}_RANGE", rng)
    ljm.eWriteName(mb, f"{pos_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(mb, f"{pos_name}_SETTLING_US", sett)
    ljm.eWriteName(mb, f"{pos_name}_EF_INDEX", 22)
    ljm.eWriteName(mb, f"{pos_name}_EF_CONFIG_A", temp)

# --------------------------------------------------------
# Configures either a transducer or a loadcell since they
# are done the same. Connects to AIN apin with range rng 
# and resolution rind. The voltage is compared to GND.
#---------------------------------------------------------
def configure_transducer_loadcell(apin, rng, rind):
    ain_name = f"AIN{apin}"
    ljm.eWriteName(mb, f"{ain_name}_NEGATIVE_CH", 199)
    ljm.eWriteName(mb, f"{ain_name}_RANGE", rng)
    ljm.eWriteName(mb, f"{ain_name}_RESOLUTION_INDEX", rind)
    ljm.eWriteName(mb, f"{ain_name}_SETTLING_US", 0)

# --------------------------------------------------------
# Configures a pin as digital I/O, with the function func.
# func MUST BE either 'input' or 'output'.
# --------------------------------------------------------
def configure_digital_io(diopin_num, func):
    if func == "input":
        ljm.eWriteName(mb, f"DIO{diopin_num}_DIRECTION", 0)
    else:
        ljm.eWriteName(mb, f"DIO{diopin_num}_DIRECTION", 1)
        ljm.eWriteName(mb, f"DIO{diopin_num}_STATE", 0)

# --------------------------------------------------------
# Configures the board clock at frequency freq.
# --------------------------------------------------------
def configure_clock(freq):
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ENABLE", 0)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_DIVISOR", 1)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ROLL_VALUE", 80000000/freq)
    ljm.eWriteName(mb, "DIO_EF_CLOCK0_ENABLE", 1)

# --------------------------------------------------------
# Configures a DIO pin to output pwm at 50 HZ and start
# value startval
# --------------------------------------------------------
def configure_pwm(diopin_num, startval):
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_ENABLE", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_INDEX", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_OPTIONS", 0)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_CONFIG_A", startval)
    ljm.eWriteName(mb, f"DIO{diopin_num}_EF_ENABLE", 1)

# ================================
# MEASUREMENT FUNCTIONS
# ================================

# --------------------------------------------------------
# Measures and returns transducer current by diving measured 
# voltage by known resistance (Ohm's Law).
# --------------------------------------------------------
def measure_transducer_current(apin, resistance):
    voltage = ljm.eReadName(mb, f"AIN{apin}")
    return voltage / resistance

# --------------------------------------------------------
# Converts transducer current to pressure and returns
# --------------------------------------------------------
def current_to_pressure(current, pmin, pmax):
    return ((current - 0.004) / 0.016) * (pmax - pmin) + pmin

# --------------------------------------------------------
# Measures the temperature by reading TC pin. Returns temp.
# --------------------------------------------------------
def read_temperature(tpin):
    return ljm.eReadName(mb, f"AIN{tpin}_EF_READ_A")

# --------------------------------------------------------
# Calculates and returns the pressure using the current 
# and conversion functions.
# --------------------------------------------------------
def read_pressure(prpin, resis, p_min, p_max):
    c = measure_transducer_current(prpin, resis)
    return current_to_pressure(c, p_min, p_max)

# --------------------------------------------------------
# Reads the load cell load using by calculating a scaling
# factor with a known load and its voltage as well as the 
# offset (0 load) voltage and using a formula with the 
# measured pin voltage.
# --------------------------------------------------------
def read_load(ldpin, v_off, kload, v_kload):
    factor = kload / (v_kload - v_off)
    pin_v = ljm.eReadName(mb, f"AIN{ldpin}")
    return (pin_v - v_off) * factor

# ================================
# DIGITAL / SERVO FUNCTIONS
# ================================

# --------------------------------------------------------
# Moves the servo motors between two positons. nextpos 
# MUST BE either 'open' or 'closed'. Returns whether motor
# moved, prints error if there is one.
# --------------------------------------------------------
def move(diopin_num, nextpos, curpos):
    if nextpos == curpos:
        print(Fore.RED + "MOVE COMMAND FAILED: ALREADY " + curpos.upper())
        return False
    if nextpos == "open":
        ljm.eWriteName(mb, f"DIO{diopin_num}_EF_CONFIG_A", openval)
        curpos = nextpos
        return True
    elif nextpos == "closed":
        ljm.eWriteName(mb, f"DIO{diopin_num}_EF_CONFIG_A", closedval)
        curpos = nextpos
        return True
    else:
        print(Fore.RED + "MOVE COMMAND FAILED: " + str(nextpos.upper()) + " INVALID MOVEMENT COMMAND")
        return False

# --------------------------------------------------------
# Detects whether a valid move command has come in. 
# move commands must be on a rising edge.
# --------------------------------------------------------    
def detect_move(pin):
    if dread(pin) == 1 and last_mc == 0:
        return True
    return False

# --------------------------------------------------------
# Controlled movement of servos.
# --------------------------------------------------------
def moveseq_start(pin1, pin2):
    global servo_state, servo_timer
    move(pin1, target)
    move(pin2, target)
    servo_state = SERVO_STAGE1
    servo_timer = time.time()

def moveseq_update(pin1, pin2):
    global servo_state
    if servo_state == SERVO_STAGE1:
        if time.time() - servo_timer >= stroke_time:
            move(pin1, next_target)
            move(pin2, next_target)
            servo_state = SERVO_STAGE2
# --------------------------------------------------------
# Reads from DIO Pin.
# --------------------------------------------------------
def dread(diopin_num):
    return ljm.eReadName(mb, f"DIO{diopin_num}")

# --------------------------------------------------------
# Writes to DIO Pin, 0 for input, 1 for output.
# --------------------------------------------------------
def dwrite(diopin_num, state):
    ljm.eWriteName(mb, f"DIO{diopin_num}_STATE", state)

# ================================
# IGNITION FUNCTIONS
# ================================

# --------------------------------------------------------
# Checks if armed, and measurements are within range before
# firing.
# --------------------------------------------------------
def is_ignition_safe(a, p, t, l, mp, mt, ml, mip, mit, mil):
    if not a:
        print(Fore.RED + "IGNITION FAILURE: SYSTEM NOT ARMED")
        return False
    if p > mp or p < mip:
        print(Fore.RED + "IGNITION FAILURE: PRESSURE OUT OF RANGE")
        return False
    if t > mt or t < mit:
        print(Fore.RED + "IGNITION FAILURE: TEMPERATURE TOO HIGH")
        return False
    if l > ml or l < mil:
        print(Fore.RED + "IGNITION FAILURE: LOAD TOO HIGH")
        return False
    return True

# --------------------------------------------------------
# Control function for firing, checking safety before
# firing and handling the result.
# --------------------------------------------------------
def fire_control(ipin, ftime, a, p, t, l, mp, mt, ml, mip, mit, mil):
    if is_ignition_safe(a, p, t, l, mp, mt, ml, mip, mit, mil):
        if lastfire == 0:
            print(Fore.YELLOW + "IGNITION ACTIVE")
            fire(ipin, ftime)
            print(Fore.YELLOW + "IGNITION COMPLETE")
            return True
        else:
            print(Fore.RED + "IGNITION FAILED: NO CONSECUTIVE IGNITES")
            return False
    else:
        return False

# --------------------------------------------------------
# Fires for time ftime.
# --------------------------------------------------------
def fire(ipin, ftime):
    dwrite(ipin, 1)
    time.sleep(ftime)
    dwrite(ipin, 0)

# ================================
# SAFETY FUNCTIONS
# ================================

# --------------------------------------------------------
# Checks if the per-sample change in a reading exceeds
# max_delta. Returns False if the rate of change is too
# large. Skips the check on the first sample.
# --------------------------------------------------------
def check_rate_of_change(history, current, max_delta):
    if timestamp == 0:
        return True
    last = history[(timestamp - 1) % hist]
    return abs(current - last) <= max_delta

# --------------------------------------------------------
# Averages current temperature with the populated history
# window and checks against fault bounds.
# --------------------------------------------------------
def check_temperature(history, temperature):
    count = min(timestamp, hist)
    avg = (np.sum(history[:count]) + temperature) / (count + 1)
    return fault_temp_min <= avg <= fault_temp_max

# --------------------------------------------------------
# Averages current pressure with the populated history
# window and checks against fault bounds.
# --------------------------------------------------------
def check_pressure(history, pressure):
    count = min(timestamp, hist)
    avg = (np.sum(history[:count]) + pressure) / (count + 1)
    return fault_pres_min <= avg <= fault_pres_max

# --------------------------------------------------------
# Averages current load with the populated history
# window and checks against fault bounds.
# --------------------------------------------------------
def check_load(history, load):
    count = min(timestamp, hist)
    avg = (np.sum(history[:count]) + load) / (count + 1)
    return fault_load_min <= avg <= fault_load_max

# --------------------------------------------------------
# Kills the current running sequence by shutting off
# a power pin, sending an abort message.
# --------------------------------------------------------
def kill(safet, safep, safel, kill_pin):
    dwrite(kill_pin, 0)
    if not safet:
        return Fore.RED + "ABORTING: TEMPERATURE UNSAFE. PLEASE RESET SYSTEM ONCE MANUALLY CONFIRMED SAFE"
    if not safep:
        return Fore.RED + "ABORTING: PRESSURE UNSAFE. PLEASE RESET SYSTEM ONCE MANUALLY CONFIRMED SAFE"
    if not safel:
        return Fore.RED + "ABORTING: LOAD UNSAFE. PLEASE RESET SYSTEM ONCE MANUALLY CONFIRMED SAFE"

def wait_for_safe_conditions():
    consecutive_safe = 0
    while consecutive_safe < REQUIRED_SAFE:
        temp = read_temperature(tpin_pos_num)
        pres = read_pressure(ppin_num, res_val, pmin, pmax)
        load = read_load(lpin_num, v_off, kload, v_kload)

        safet = fault_temp_min <= temp <= fault_temp_max
        safep = fault_pres_min <= pres <= fault_pres_max
        safel = fault_load_min <= load <= fault_load_max

        if safet and safep and safel:
            consecutive_safe += 1
        else:
            consecutive_safe = 0
        time.sleep(0.1)

def wait_for_arm():
    global killed, pastt, pastp, pastl
    while not dread(arm_pin):
        pass
    pastt[:] = 0
    pastp[:] = 0
    pastl[:] = 0
    dwrite(kill_pin_num, 1)
    killed = False
        
# ================================
# CONFIGURATION
# ================================

# Configure Temperature Sensing
configure_thermocouple(tpin_pos_num, tpin_neg_num, trng, trind, tset, celsius)
# Configure Pressure Sensing
configure_transducer_loadcell(ppin_num, prng, prind)
# Configure Load Sensing
configure_transducer_loadcell(lpin_num, lrng, lrind)
# Configure the Servo Pins
configure_clock(pwm_freq)
configure_digital_io(servo1_pwm_pin, "output")
configure_pwm(servo1_pwm_pin, startval)
configure_digital_io(servo2_pwm_pin, "output")
configure_pwm(servo2_pwm_pin, startval)
configure_digital_io(movact_pin_num, "input")
# Configure ignition pins
configure_digital_io(fire_pin, "input")
configure_digital_io(ignite_out_pin, "output")

# ================================
# MAIN LOOP 
# ================================

while True:
    temp = read_temperature(tpin_pos_num)
    pres = read_pressure(ppin_num, res_val, pmin, pmax)
    load = read_load(lpin_num, v_off, kload, v_kload)

    # Rate-of-change fault check
    safe_temp_rate = check_rate_of_change(pastt, temp, max_temp_rate)
    safe_pres_rate = check_rate_of_change(pastp, pres, max_pres_rate)
    safe_load_rate = check_rate_of_change(pastl, load, max_load_rate)

    # Two-tier threshold checks
    in_fault_temp = not (fault_temp_min <= temp <= fault_temp_max)
    in_fault_pres = not (fault_pres_min <= pres <= fault_pres_max)
    in_fault_load = not (fault_load_min <= load <= fault_load_max)
    in_warn_temp  = not (warn_temp_min  <= temp <= warn_temp_max)
    in_warn_pres  = not (warn_pres_min  <= pres <= warn_pres_max)
    in_warn_load  = not (warn_load_min  <= load <= warn_load_max)

    safet = not in_fault_temp and safe_temp_rate and check_temperature(pastt, temp)
    safep = not in_fault_pres and safe_pres_rate and check_pressure(pastp, pres)
    safel = not in_fault_load and safe_load_rate and check_load(pastl, load)

    if not (safet and safep and safel):
        kill_message = kill(safet, safep, safel, kill_pin_num)
        killed = True
        print(kill_message)
        wait_for_safe_conditions()
        print(Fore.GREEN + "SYSTEM SAFE: ARM TO RESTART")
        wait_for_arm()
    elif in_warn_temp or in_warn_pres or in_warn_load:
        print(Fore.YELLOW + "WARNING: APPROACHING SAFETY LIMITS")

    if not killed:
        idx = timestamp%hist
        pastt[idx] = temp
        pastp[idx] = pres
        pastl[idx] = load
        armed = (dread(arm_pin) == 1)
        firing = (dread(fire_pin) == 1)

        if firing:
            if timestamp >= hist:
                did_fire = fire_control(ignite_out_pin, fire_time, armed, pres, temp, load, fault_pres_max, fault_temp_max, fault_load_max, fault_pres_min, fault_temp_min, fault_load_min)
                lastfire = int(did_fire)
            else:
                print(Fore.RED + "IGNITION FAILED: SYSTEM NOT READY")
                lastfire = 0
        
        movcom = detect_move(movact_pin_num)
        if movcom and servo_state == SERVO_IDLE:
            moveseq_start(servo1_pwm_pin, servo2_pwm_pin)
            moveseq_update(servo1_pwm_pin, servo2_pwm_pin)

        print(Fore.GREEN + "Temperature (C):", temp)
        print(Fore.GREEN + "Pressure (psi):", pres)
        print(Fore.GREEN + "Force:", load)

        timestamp += 1
        time.sleep(0.1)