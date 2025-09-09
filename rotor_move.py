#!/usr/bin/env python3
import time, threading, signal, sys
from flask import Flask, request, jsonify

import pigpio

# ===== PIN MAP (BCM) =====
# Left motor (PWM0 channel)
L_RPWM, L_LPWM = 12, 18
# Right motor (PWM1 channel)
R_RPWM, R_LPWM = 13, 19

PWM_FREQ   = 8000     # try 2k/8k/20k if you want
DEADTIME_S = 0.06     # brief deadtime when flipping direction

# Serve files from ./static (so / -> static/index.html automatically)
app = Flask(__name__, static_folder="static", static_url_path="")

pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpio daemon not running. Do: sudo systemctl start pigpiod", file=sys.stderr)
    sys.exit(1)

for p in (L_RPWM, L_LPWM, R_RPWM, R_LPWM):
    pi.set_mode(p, pigpio.OUTPUT)

def _hw_pwm(pin, duty_pct):
    duty_pct = max(0.0, min(100.0, float(duty_pct)))
    pi.hardware_PWM(pin, PWM_FREQ, int(duty_pct * 10000))  # 0..1_000_000

def _stop_pair(a, b):
    pi.hardware_PWM(a, 0, 0)
    pi.hardware_PWM(b, 0, 0)

def stop_all():
    _stop_pair(L_RPWM, L_LPWM)
    _stop_pair(R_RPWM, R_LPWM)

last_sign = {"L": 0, "R": 0}  # for deadtime only when direction flips

def set_motor(side: str, dc: float):
    """dc in [-100..100]; + = forward, - = reverse. Only one pin active per side."""
    global last_sign
    sign = 0 if abs(dc) < 1e-3 else (1 if dc > 0 else -1)

    if side == "L":
        if sign != 0 and sign != last_sign["L"]:
            _stop_pair(L_RPWM, L_LPWM); time.sleep(DEADTIME_S)
        if dc >= 0:
            _hw_pwm(L_LPWM, 0); _hw_pwm(L_RPWM, dc)
        else:
            _hw_pwm(L_RPWM, 0); _hw_pwm(L_LPWM, -dc)
        last_sign["L"] = sign
    elif side == "R":
        if sign != 0 and sign != last_sign["R"]:
            _stop_pair(R_RPWM, R_LPWM); time.sleep(DEADTIME_S)
        if dc >= 0:
            _hw_pwm(R_LPWM, 0); _hw_pwm(R_RPWM, dc)
        else:
            _hw_pwm(R_RPWM, 0); _hw_pwm(R_LPWM, -dc)
        last_sign["R"] = sign

def drive(v_pct: float, yaw_pct: float):
    """v_pct: -100..100 (neg=reverse), yaw_pct: -100..100 (pos=left, neg=right)."""
    L = v_pct - yaw_pct
    R = v_pct + yaw_pct
    m = max(1.0, max(abs(L), abs(R)) / 100.0)
    L = max(-100, min(100, L / m * 100))
    R = max(-100, min(100, R / m * 100))
    set_motor("L", L)
    set_motor("R", R)

# --------- State & control loop (applies latest command at ~30Hz) ----------
state = {"v": 0.0, "yaw": 0.0, "estop": False}
lock = threading.Lock()
running = True

def control_loop():
    period = 1.0 / 30.0
    while running:
        with lock:
            if state["estop"]:
                stop_all()
            else:
                if abs(state["v"]) < 1e-3 and abs(state["yaw"]) < 1e-3:
                    stop_all()
                else:
                    drive(state["v"], state["yaw"])
        time.sleep(period)

threading.Thread(target=control_loop, daemon=True).start()

# --------- API ---------

# Static index at '/' (served automatically from ./static)

@app.post("/command")
def command():
    """
    Body:
      mode: 'forward'|'reverse'|'left'|'right'|'custom'
      speed: 0..100  (for forward/reverse)
      yaw:   0..100  (for left/right)
      custom: v and yaw (-100..100) if mode=='custom'
    """
    data = request.get_json(force=True, silent=True) or {}
    mode = (data.get("mode") or "").lower()
    v, yaw = 0.0, 0.0

    if mode == "forward":
        v = max(0.0, min(100.0, float(data.get("speed") or 0.0)))
    elif mode == "reverse":
        v = -max(0.0, min(100.0, float(data.get("speed") or 0.0)))
    elif mode == "left":
        yaw =  max(0.0, min(100.0, float(data.get("yaw") or 0.0)))
    elif mode == "right":
        yaw = -max(0.0, min(100.0, float(data.get("yaw") or 0.0)))
    elif mode == "custom":
        v   = float(data.get("v") or 0.0)
        yaw = float(data.get("yaw") or 0.0)
        v   = max(-100.0, min(100.0, v))
        yaw = max(-100.0, min(100.0, yaw))
    else:
        return jsonify(ok=False, error="bad mode"), 400

    with lock:
        if state["estop"]:
            return jsonify(ok=False, error="estop engaged"), 409
        state["v"], state["yaw"] = v, yaw
    return jsonify(ok=True, v=v, yaw=yaw)

@app.post("/stop")
def stop_cmd():
    with lock:
        state["v"], state["yaw"] = 0.0, 0.0
    stop_all()
    return jsonify(ok=True)

@app.post("/estop")
def estop_cmd():
    with lock:
        state["estop"] = True
        state["v"], state["yaw"] = 0.0, 0.0
    stop_all()
    return jsonify(ok=True)

@app.post("/reset")
def reset_cmd():
    with lock:
        state["estop"] = False
        state["v"], state["yaw"] = 0.0, 0.0
    stop_all()
    return jsonify(ok=True)

@app.get("/status")
def status():
    with lock:
        return jsonify(dict(state))

def cleanup(*_):
    global running
    running = False
    try:
        stop_all()
    finally:
        pi.stop()
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

if __name__ == "__main__":
    # Serve static/index.html at http://<pi-ip>:5000/
    app.run(host="0.0.0.0", port=5000)
