# LPC LABJACK T7 PYTHON CONFIG SCRIPT
# Author: Thomas Tedeschi
# Last Update Date: 4/25/2026

import threading
import time
import numpy as np
from colorama import Fore

import daqsampling
import daqservos

# Both modules open the LabJack at import time; unify to a single handle
daqsampling.mb = daqservos.mb

# Wire daqsampling's abort helper to the servo-side hardware functions
# (daqsampling.abort calls these, but they're only defined in daqservos)
daqsampling.move       = daqservos.move
daqsampling.v6_release = daqservos.v6_release
daqsampling.fire_off   = daqservos.fire_off
daqsampling.dwrite     = daqservos.dwrite

# Wire daqservos's hot-fire pressure read to daqsampling's measurement functions
daqservos.read_pressure = daqsampling.read_pressure

HIST = daqsampling.HIST

# ================================
# THREAD SYNCHRONIZATION
# ================================
sample_lock = threading.Lock()   # guards snapshot reads/writes
abort_event = threading.Event()  # set on any fault or interrupt


# ================================
# SHARED SENSOR SNAPSHOT
# ================================
class SensorSnapshot:
    def __init__(self):
        self.tc1  = 0.0
        self.tc2  = 0.0
        self.pt1  = 0.0
        self.pt2  = 0.0
        self.pt3  = 0.0
        self.pt4  = 0.0
        self.load = 0.0

snapshot = SensorSnapshot()


# ================================
# ABORT BRIDGE
# Replaces daqsampling.abort so a safety fault in the sampling thread
# immediately fires the servo-side hardware abort and signals both threads.
# ================================
def _abort_and_signal(reason=""):
    daqservos.abort(reason)
    abort_event.set()

daqsampling.abort = _abort_and_signal


# ================================
# THREADS
# ================================

def sample_thread():
    while not abort_event.is_set():
        # Keep sampling module's view of FSM state current with servo FSM
        daqsampling.system_state = daqservos.system_state

        result = daqsampling.sample_safety_check()
        tc1, tc2, pt1, pt2, pt3, pt4, load = result[:7]

        with sample_lock:
            snapshot.tc1  = tc1
            snapshot.tc2  = tc2
            snapshot.pt1  = pt1
            snapshot.pt2  = pt2
            snapshot.pt3  = pt3
            snapshot.pt4  = pt4
            snapshot.load = load

        time.sleep(0.01)   # 100 Hz sampling rate


def servo_thread():
    while not abort_event.is_set():
        with sample_lock:
            tc1  = snapshot.tc1
            tc2  = snapshot.tc2
            pt1  = snapshot.pt1
            pt2  = snapshot.pt2
            pt3  = snapshot.pt3
            pt4  = snapshot.pt4

        state = daqservos.system_state
        if state == daqservos.STATE_COLD_OPS:
            daqservos.handle_cold_ops(tc1, tc2, pt1, pt2, pt3, pt4)
        elif state == daqservos.STATE_PRE_FIRE_PURGE:
            daqservos.handle_pre_fire_purge()          # blocks for Purge_Time
        elif state == daqservos.STATE_FILL:
            daqservos.handle_fill(pt2)
        elif state == daqservos.STATE_STATE_CHECK:
            daqservos.handle_state_check(tc1, tc2, pt1, pt2, pt3, pt4)
        elif state == daqservos.STATE_HOT_FIRE:
            daqservos.handle_hot_fire()                # blocks for full sequence
        elif state == daqservos.STATE_POST_FIRE_PURGE:
            daqservos.handle_post_fire_purge(pt3)
        elif state == daqservos.STATE_VENT_SAFING:
            daqservos.handle_vent_safing(tc1, tc2, pt1, pt2, pt3, pt4)
        elif state == daqservos.STATE_ABORT:
            abort_event.set()

        time.sleep(0.001)  # FSM ticks at ~1 kHz between non-blocking states


# ================================
# ENTRY POINT
# ================================
if __name__ == "__main__":
    from labjack import ljm

    t_sample = threading.Thread(target=sample_thread, daemon=True, name="sampling")
    t_servo  = threading.Thread(target=servo_thread,  daemon=True, name="servos")

    t_sample.start()
    t_servo.start()

    try:
        while not abort_event.is_set():
            abort_event.wait(timeout=0.1)
    except KeyboardInterrupt:
        print(Fore.RED + "KEYBOARD INTERRUPT — ABORTING")
        _abort_and_signal("KEYBOARD INTERRUPT")
    finally:
        abort_event.set()
        t_sample.join(timeout=2.0)
        t_servo.join(timeout=2.0)
        ljm.close(daqservos.mb)
