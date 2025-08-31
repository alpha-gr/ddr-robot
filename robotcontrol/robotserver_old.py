# ws_motor_server.py
import asyncio
import RPi.GPIO as GPIO
import websockets

# GPIO (BCM)
M1_IN1 = 19
M1_IN2 = 26
M2_IN1 = 16
M2_IN2 = 20

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for p in (M1_IN1, M1_IN2, M2_IN1, M2_IN2):
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.LOW)

def stop_all():
    for p in (M1_IN1, M1_IN2, M2_IN1, M2_IN2):
        GPIO.output(p, GPIO.LOW)

def forward():
    GPIO.output(M1_IN1, GPIO.HIGH); GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN1, GPIO.HIGH); GPIO.output(M2_IN2, GPIO.LOW)

def backward():
    GPIO.output(M1_IN1, GPIO.LOW);  GPIO.output(M1_IN2, GPIO.HIGH)
    GPIO.output(M2_IN1, GPIO.LOW);  GPIO.output(M2_IN2, GPIO.HIGH)

def left():
    # giri in loco: motore sinistro indietro, destro avanti (aggiusta in base a orientamento)
    GPIO.output(M1_IN1, GPIO.HIGH);  GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN1, GPIO.LOW); GPIO.output(M2_IN2, GPIO.HIGH)

def right():
    GPIO.output(M1_IN1, GPIO.LOW); GPIO.output(M1_IN2, GPIO.HIGH)
    GPIO.output(M2_IN1, GPIO.HIGH);  GPIO.output(M2_IN2, GPIO.LOW)

COMMANDS = {
    "forward": forward,
    "backward": backward,
    "left": left,
    "right": right,
    "stop": stop_all
}

async def handler(ws):
    print("Connessione da", ws.remote_address)
    try:
        async for msg in ws:
            msg = msg.strip().lower()
            print("CMD ricevuto:", msg)
            if msg in COMMANDS:
                COMMANDS[msg]()
                await ws.send("ok")
            else:
                await ws.send("unknown")
    except Exception as e:
        print("Connessione chiusa:", e)
    finally:
        stop_all()

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("WebSocket server in ascolto su 0.0.0.0:8765")
        await asyncio.Future()

try:
    asyncio.run(main())
finally:
    stop_all()
    GPIO.cleanup()
