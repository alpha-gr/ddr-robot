"""
Client di test per controllo remoto del robot
"""
import asyncio
import websockets
import json
import time

async def test_robot_control():
    uri = "ws://localhost:8765"
    print(f"Connessione a {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("🔗 Connesso al robot!")
            
            # Test 1: Engage (attiva controllo remoto)
            print("\n=== TEST 1: ENGAGE ===")
            engage_msg = {"command": "engage"}
            await websocket.send(json.dumps(engage_msg))
            response = await websocket.recv()
            print(f"Engage response: {json.loads(response)}")
            
            # Aspetta un po'
            await asyncio.sleep(2)
            
            # Test 2: Move robot (con pathfinding)
            print("\n=== TEST 2: MOVE ROBOT ===")
            move_msg = {
                "command": "gototarget",
                "target": "Slot_2"
            }
            await websocket.send(json.dumps(move_msg))
            response = await websocket.recv()
            move_response = json.loads(response)
            print(f"Move response: {move_response}")
            
            movement_id = move_response.get("movement_id")
            if movement_id:
                print(f"🎯 Movimento iniziato con ID: {movement_id}")

                # invio del messaggio di pausa
                await asyncio.sleep(1)
                pause_msg = {"command": "pause"}
                await websocket.send(json.dumps(pause_msg))
                pause_response = await websocket.recv() 
                print(f"Pause response: {json.loads(pause_response)}")
                await asyncio.sleep(2)  # pausa di 2 secondi
                # invio del messaggio di ripresa
                resume_msg = {"command": "resume"}
                await websocket.send(json.dumps(resume_msg))
                resume_response = await websocket.recv()
                print(f"Resume response: {json.loads(resume_response)}")  
                
                # Ascolta per il messaggio moverobotdone
                print("⏳ Aspettando completion del movimento...")
                start_time = time.time()
                timeout = 30  # 30 secondi timeout
                
                while time.time() - start_time < timeout:
                    try:
                        # Ricevi messaggi con timeout
                        done_response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                        data = json.loads(done_response)
                        
                        if data.get("command") == "moverobotdone":
                            print(f"✅ Movimento completato: {data}")
                            break
                        else:
                            print(f"📊 Status update: {data.get('type', 'unknown')}")
                            
                    except asyncio.TimeoutError:
                        # Continua ad aspettare
                        continue
                else:
                    print("⚠️ Timeout: movimento non completato entro 30 secondi")
            
            # Test 3: Secondo movimento
            print("\n=== TEST 3: SECONDO MOVIMENTO ===")
            move_msg2 = {
                "command": "moverobot",
                "x": 20.0,
                "y": 60.0
            }
            await websocket.send(json.dumps(move_msg2))
            response2 = await websocket.recv()
            move_response2 = json.loads(response2)
            print(f"Move response 2: {move_response2}")
            
            # Aspetta completion del secondo movimento
            movement_id2 = move_response2.get("movement_id")
            if movement_id2:
                print(f"🎯 Secondo movimento ID: {movement_id2}")
                print("⏳ Aspettando completion del secondo movimento...")
                
                start_time = time.time()
                while time.time() - start_time < timeout:
                    try:
                        done_response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                        data = json.loads(done_response)
                        
                        if data.get("command") == "moverobotdone":
                            print(f"✅ Secondo movimento completato: {data}")
                            break
                            
                    except asyncio.TimeoutError:
                        continue
                else:
                    print("⚠️ Timeout: secondo movimento non completato")
            
            # Test 4: Disengage
            print("\n=== TEST 4: DISENGAGE ===")
            disengage_msg = {"command": "disengage"}
            await websocket.send(json.dumps(disengage_msg))
            response = await websocket.recv()
            print(f"Disengage response: {json.loads(response)}")
            
            print("\n✅ Test completato!")
            
    except websockets.exceptions.ConnectionRefusedError:
        print("❌ Connessione rifiutata. Assicurati che il robot server sia in esecuzione.")
    except Exception as e:
        print(f"❌ Errore durante il test: {e}")

async def test_basic_commands():
    """Test semplificato solo per comandi base"""
    uri = "ws://localhost:8765"
    
    try:
        async with websockets.connect(uri) as websocket:
            print("🔗 Connesso per test base!")
            
            # Engage
            await websocket.send(json.dumps({"command": "engage"}))
            print("Engage:", json.loads(await websocket.recv()))
            
            # Move
            await websocket.send(json.dumps({
                "command": "moverobot", 
                "x": 30.0, 
                "y": 40.0
            }))
            print("Move:", json.loads(await websocket.recv()))
            
            await asyncio.sleep(3)
            
            # Disengage
            await websocket.send(json.dumps({"command": "disengage"}))
            print("Disengage:", json.loads(await websocket.recv()))
            
    except Exception as e:
        print(f"Errore test base: {e}")

if __name__ == "__main__":
    print("=== Test Client Robot WebSocket ===")
    print("1. Test completo con monitoring moverobotdone")
    print("2. Test base solo comandi")
    
    choice = input("Scegli test (1/2): ").strip()
    
    if choice == "1":
        asyncio.run(test_robot_control())
    elif choice == "2":
        asyncio.run(test_basic_commands())
    else:
        print("Scelta non valida")
