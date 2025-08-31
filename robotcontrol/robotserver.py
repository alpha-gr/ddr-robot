#!/usr/bin/env python3
"""
ws_motor_server_pwm.py
WebSocket server per joystick (x,y) -> motors via L298N (Raspberry Pi).
Riceve JSON {"x":..., "y":...} oppure comandi plain text (stop, forward, ...).

Dipendenze:
    sudo pip3 install websockets RPi.GPIO
Eseguire con sudo se necessario.
"""

import asyncio
import json
import signal
import sys
from contextlib import suppress

import RPi.GPIO as GPIO
import websockets

# ---------------- CONFIG ----------------
# PIN (BCM)
M1_IN1 = 19
M1_IN2 = 26
M2_IN1 = 16
M2_IN2 = 20

ENA_PIN = 18   # PWM per motore sinistro (M1)
ENB_PIN = 13   # PWM per motore destro (M2)

MIN_DUTY_CYCLE = 30
PWM_FREQ = 50        # Hz
SAFETY_TIMEOUT = 1.0   # s -> stop se non riceve comandi
DEADZONE = 0.01         # modulo sotto cui consideriamo zero
WS_HOST = "0.0.0.0"
WS_PORT = 8765
# ----------------------------------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# setup direzione pins
for p in (M1_IN1, M1_IN2, M2_IN1, M2_IN2):
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

# setup PWM (ENA/ENB). Se non collegati, lasciali None e la funzione userà on/off
_pwm_ena = None
_pwm_enb = None
if ENA_PIN is not None:
    GPIO.setup(ENA_PIN, GPIO.OUT)
    _p = GPIO.PWM(ENA_PIN, PWM_FREQ)
    _p.start(0)
    _pwm_ena = _p
if ENB_PIN is not None:
    GPIO.setup(ENB_PIN, GPIO.OUT)
    _p = GPIO.PWM(ENB_PIN, PWM_FREQ)
    _p.start(0)
    _pwm_enb = _p

# stato per watchdog
last_cmd_ts = 0.0
last_cmd_lock = asyncio.Lock()

# helpers
def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))

def _set_direction_and_power(in1, in2, pwm_obj, speed):
    """
    speed: -1..1, >0 = forward, <0 = backward. pwm_obj may be None.
    If |speed| < DEADZONE -> stop (both low).
    """
    mag = abs(speed)
    if mag < DEADZONE:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        if pwm_obj is not None:
            pwm_obj.ChangeDutyCycle(0)
        return

    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

    duty = min(100.0, MIN_DUTY_CYCLE + (mag * (100 - MIN_DUTY_CYCLE)))
    print(f"[motor] set {in1} {in2} duty={duty}")
    if pwm_obj is not None:
        pwm_obj.ChangeDutyCycle(duty)
    else:
        # no PWM available: emulate on/off
        GPIO.output(in1, GPIO.HIGH if speed > 0 else GPIO.LOW)
        GPIO.output(in2, GPIO.LOW if speed > 0 else GPIO.HIGH)

def stop_all():
    for p in (M1_IN1, M1_IN2, M2_IN1, M2_IN2):
        GPIO.output(p, GPIO.LOW)
    if _pwm_ena is not None:
        _pwm_ena.ChangeDutyCycle(0)
    if _pwm_enb is not None:
        _pwm_enb.ChangeDutyCycle(0)

# discrete commands (kept for compatibility)
def cmd_forward():
    _set_direction_and_power(M1_IN1, M1_IN2, _pwm_ena, 1.0)
    _set_direction_and_power(M2_IN1, M2_IN2, _pwm_enb, 1.0)

def cmd_backward():
    _set_direction_and_power(M1_IN1, M1_IN2, _pwm_ena, -1.0)
    _set_direction_and_power(M2_IN1, M2_IN2, _pwm_enb, -1.0)

def cmd_left():
    # rotate in place: left backward, right forward (tweak if orientation different)
    _set_direction_and_power(M1_IN1, M1_IN2, _pwm_ena, -1.0)
    _set_direction_and_power(M2_IN1, M2_IN2, _pwm_enb,  1.0)

def cmd_right():
    _set_direction_and_power(M1_IN1, M1_IN2, _pwm_ena,  1.0)
    _set_direction_and_power(M2_IN1, M2_IN2, _pwm_enb, -1.0)

COMMANDS = {
    "forward": cmd_forward,
    "backward": cmd_backward,
    "left": cmd_left,
    "right": cmd_right,
    "stop": stop_all,
}

async def safety_watchdog():
    """Ferma motori se non riceve comandi entro SAFETY_TIMEOUT"""
    global last_cmd_ts
    while True:
        await asyncio.sleep(0.1)
        async with last_cmd_lock:
            ts = last_cmd_ts
        if ts != 0 and (asyncio.get_event_loop().time() - ts) > SAFETY_TIMEOUT:
            stop_all()

async def process_json_xy(x, y):
    """
    x,y in -1..1.
    y == -1 => forward full (this matches nipple.js example).
    Compute forward = -y (so forward positive), then differential:
      v_l = forward + x
      v_r = forward - x
    Clamp to [-1,1], set motors.
    """
    # map joystick convention: y = -1 -> forward, so forward = -y
    forward = float(y)
    lateral = - float(x)
    # compute
    v_l = forward + lateral
    v_r = forward - lateral
    v_l = _clamp(v_l)
    v_r = _clamp(v_r)
    # apply
    _set_direction_and_power(M1_IN1, M1_IN2, _pwm_ena, v_l)
    _set_direction_and_power(M2_IN1, M2_IN2, _pwm_enb, v_r)

async def process_message(msg: str) -> str:
    """Accetta:
       - JSON: {"x":..., "y":...}  oppure {"cmd":"set_speed","m1":..,"m2":..}
       - plain: 'forward','stop',...
    """
    global last_cmd_ts
    # try JSON
    try:
        payload = json.loads(msg)
    except Exception:
        cmd = msg.strip().lower()
        if not cmd:
            return "noop"
        if cmd in COMMANDS:
            COMMANDS[cmd]()
            async with last_cmd_lock:
                last_cmd_ts = asyncio.get_event_loop().time()
            return "ok"
        return "error: unknown command"

    # JSON path
    if isinstance(payload, dict):
        if "x" in payload and "y" in payload:
            try:
                x = float(payload["x"])
                y = float(payload["y"])
            except Exception:
                return "error: invalid xy"
            # clamp inputs
            x = _clamp(x)
            y = _clamp(y)
            await process_json_xy(x, y)
            async with last_cmd_lock:
                last_cmd_ts = asyncio.get_event_loop().time()
            return f"ok xy x={x} y={y}"
        cmd = payload.get("cmd", "").lower()
        if cmd == "set_speed":
            # optional: set both PWM duty directly (0..100)
            try:
                m1 = float(payload.get("m1", 100.0))
                m2 = float(payload.get("m2", 100.0))
            except Exception:
                return "error: invalid speeds"
            if _pwm_ena is not None:
                _pwm_ena.ChangeDutyCycle(max(0.0, min(100.0, m1)))
            if _pwm_enb is not None:
                _pwm_enb.ChangeDutyCycle(max(0.0, min(100.0, m2)))
            async with last_cmd_lock:
                last_cmd_ts = asyncio.get_event_loop().time()
            return f"ok set_speed m1={m1} m2={m2}"
        if cmd in COMMANDS:
            COMMANDS[cmd]()
            async with last_cmd_lock:
                last_cmd_ts = asyncio.get_event_loop().time()
            return "ok"
        return "error: unknown json cmd"
    return "error: bad payload"

async def handler(websocket):
    peer = websocket.remote_address
    print(f"[ws] connessione da {peer}")
    try:
        async for message in websocket:
            if not message:
                continue
            # message can be plain or JSON string
            # allow both raw JSON object or stringified JSON
            try:
                msg = message.strip()
            except Exception:
                msg = message
            print(f"[ws] msg: {msg}")
            resp = await process_message(msg)
            with suppress(Exception):
                await websocket.send(resp)
    except websockets.ConnectionClosedOK:
        print(f"[ws] chiusa normale {peer}")
    except Exception as e:
        print(f"[ws] eccezione handler: {e}")
    finally:
        stop_all()
        print(f"[ws] cleanup per {peer}")

async def main():
    stop_all()
    server = await websockets.serve(handler, WS_HOST, WS_PORT)
    print(f"[main] WS server su ws://{WS_HOST}:{WS_PORT}")
    wd = asyncio.create_task(safety_watchdog())

    # handle signals
    stop_event = asyncio.Event()
    def _on_sig(signum, frame):
        print("[main] segnale ricevuto, chiudo.")
        stop_event.set()
    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    await stop_event.wait()
    wd.cancel()
    server.close()
    await server.wait_closed()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        stop_all()
        if _pwm_ena is not None:
            _pwm_ena.stop()
        if _pwm_enb is not None:
            _pwm_enb.stop()
        GPIO.cleanup()
        print("[main] GPIO cleaned. uscita.")
