"""
WebSocket Server per controllo remoto del robot
"""
import asyncio
import json
import websockets
import logging
from typing import Optional, Callable, Dict, Any

class RobotWebSocketServer:
    def __init__(self, host: str = "localhost", port: int = 8765):
        self.host = host
        self.port = port
        self.server = None
        self.clients = set()
        self.message_handler: Optional[Callable] = None
        self.logger = logging.getLogger(__name__)
    
    def set_message_handler(self, handler: Callable):
        """Imposta il gestore dei messaggi"""
        self.message_handler = handler
    
    async def handle_client(self, websocket):
        """Gestisce connessione client"""
        self.clients.add(websocket)
        self.logger.info(f"Client connesso: {websocket.remote_address}")
        
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    if self.message_handler:
                        response = await self.message_handler(data)
                        await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({"error": "Invalid JSON"}))
                except Exception as e:
                    await websocket.send(json.dumps({"error": str(e)}))
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.clients.remove(websocket)
            self.logger.info(f"Client disconnesso")
    
    async def start(self):
        """Avvia il server"""
        self.server = await websockets.serve(
            self.handle_client, self.host, self.port
        )
        self.logger.info(f"WebSocket server avviato su {self.host}:{self.port}")
    
    async def stop(self):
        """Ferma il server"""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
    
    async def broadcast(self, message: Dict[str, Any]):
        """Invia messaggio a tutti i client connessi"""
        if self.clients:
            await asyncio.gather(
                *[client.send(json.dumps(message)) for client in self.clients],
                return_exceptions=True
            )