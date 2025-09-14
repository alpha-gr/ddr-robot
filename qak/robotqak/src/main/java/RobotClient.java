package main.java;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.net.URI;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * Client WebSocket sincrono e semplificato per il robot DDR
 */
public class RobotClient {
    
    private static final Logger logger = LoggerFactory.getLogger(RobotClient.class);
    
    private WebSocketClient client;
    private final Gson gson = new Gson();
    
    // Coda solo per risposte immediate
    private final BlockingQueue<JsonObject> responseQueue = new LinkedBlockingQueue<>();
    
    // Stati semplici
    private boolean connected = false;
    private boolean engaged = false;
    
    /**
     * Costruttore
     */
    public RobotClient(String serverUri) {
        try {
            URI uri = URI.create(serverUri);
            initializeClient(uri);
        } catch (Exception e) {
            throw new RuntimeException("Impossibile inizializzare client: " + e.getMessage(), e);
        }
    }
    
    private void initializeClient(URI serverUri) {
        client = new WebSocketClient(serverUri) {
            @Override
            public void onOpen(ServerHandshake handshake) {
                connected = true;
                logger.info("✅ Connesso al robot");
            }
            
            @Override
            public void onMessage(String message) {
                try {
                    JsonObject json = JsonParser.parseString(message).getAsJsonObject();
                    logger.debug("📥 Ricevuto: {}", message);
                    
                    // Tutte le risposte vanno nella coda unica
                    responseQueue.offer(json);
                    
                } catch (Exception e) {
                    logger.error("Errore parsing messaggio: {}", message, e);
                }
            }
            
            @Override
            public void onClose(int code, String reason, boolean remote) {
                connected = false;
                engaged = false;
                logger.info("❌ Disconnesso: {}", reason);
            }
            
            @Override
            public void onError(Exception ex) {
                logger.error("🚨 Errore WebSocket", ex);
            }
        };
    }
    
    /**
     * Connette al server (sincrono)
     */
    public void connect() throws Exception {
        client.connect();
        
        // Aspetta connessione per max 5 secondi
        for (int i = 0; i < 50; i++) {
            if (connected) return;
            Thread.sleep(100);
        }
        
        throw new RuntimeException("Timeout connessione");
    }
    
    /**
     * Disconnette dal server
     */
    public void disconnect() {
        if (client != null) {
            client.close();
        }
    }
    
    /**
     * Attiva il controllo remoto (sincrono)
     */
    public void engage() throws Exception {
        JsonObject command = new JsonObject();
        command.addProperty("command", "engage");
        
        JsonObject response = sendCommandAndWaitResponse(command);
        
        if ("success".equals(response.get("status").getAsString())) {
            engaged = true;
            logger.info("🔗 Robot ATTIVATO");
        } else {
            throw new RuntimeException("Errore engage: " + response.get("message").getAsString());
        }
    }
    
    /**
     * Disattiva il controllo remoto (sincrono)
     */
    public void disengage() throws Exception {
        JsonObject command = new JsonObject();
        command.addProperty("command", "disengage");
        
        JsonObject response = sendCommandAndWaitResponse(command);
        
        if ("success".equals(response.get("status").getAsString())) {
            engaged = false;
            logger.info("🔗 Robot DISATTIVATO");
        } else {
            throw new RuntimeException("Errore disengage: " + response.get("message").getAsString());
        }
    }
    
    /**
     * Mette in pausa il robot (sincrono)
     */
    public void pause() throws Exception {
        JsonObject command = new JsonObject();
        command.addProperty("command", "pause");
        System.out.println("robot paused");
        
        JsonObject response = sendCommandAndWaitResponse(command);
        
        if ("success".equals(response.get("status").getAsString())) {
            logger.info("robot in pausa");
        } else {
            throw new RuntimeException("Errore pause: " + response.get("message").getAsString());
        }
    }
    
    /**
     * Riattiva il robot dopo la pausa (sincrono)
     */
    public void resume() throws Exception {
        JsonObject command = new JsonObject();
        command.addProperty("command", "resume");
        
        JsonObject response = sendCommandAndWaitResponse(command);
        
        if ("success".equals(response.get("status").getAsString())) {
            logger.info("robot riattivato");
        } else {
            throw new RuntimeException("Errore resume: " + response.get("message").getAsString());
        }
    }
    
    /**
     * Muove il robot e aspetta il completamento (sincrono completo)
     */
    public String moveRobot(int x, int y) throws Exception {
        if (!engaged) {
            throw new IllegalStateException("Robot non attivato. Chiamare engage() prima.");
        }
        
        JsonObject command = new JsonObject();
        command.addProperty("command", "movetosquare");
        command.addProperty("x", x);
        command.addProperty("y", y);
        
        // Fase 1: Invia comando e aspetta conferma avvio
        JsonObject response = sendCommandAndWaitResponse(command);
        
        if (!"success".equals(response.get("status").getAsString())) {
            throw new RuntimeException("Errore movimento: " + response.get("message").getAsString());
        }
        
        String movementId = response.get("movement_id").getAsString();
        logger.info("🎯 Movimento avviato: ({}, {}) [ID: {}]", x, y, movementId);
        
        // Fase 2: Aspetta completamento movimento
        logger.info("⏳ Aspettando completamento movimento...");
        waitForMovementCompletion(movementId);
        
        logger.info("✅ Movimento completato!");
        return movementId;
    }
    
    /**
     * Aspetta che arrivi il messaggio moverobotdone per un movimento specifico
     */
    private void waitForMovementCompletion(String expectedMovementId) throws Exception {
        while (true) {
            // Aspetta messaggio
            JsonObject message = responseQueue.poll(3, TimeUnit.MINUTES);
            
            if (message == null) {
                throw new RuntimeException("Timeout: movimento non completato entro il timeout");
            }
            
            // Controlla se è un messaggio moverobotdone
            if (message.has("command") && "moverobotdone".equals(message.get("command").getAsString())) {
                String receivedMovementId = message.has("movement_id") ? 
                    message.get("movement_id").getAsString() : "unknown";
                
                if (expectedMovementId.equals(receivedMovementId)) {
                    // Movimento completato correttamente
                    logger.info("📨 Ricevuto moverobotdone per movimento: {}", receivedMovementId);
                    return;
                } else {
                    logger.warn("⚠️ Ricevuto moverobotdone per movimento diverso: atteso {}, ricevuto {}", 
                               expectedMovementId, receivedMovementId);
                    // Continua ad aspettare il movimento corretto
                }
            } else {
                // Non è moverobotdone, rimetti in coda per altri usi
                logger.debug("📥 Messaggio non moverobotdone ricevuto durante attesa: {}", message);
                // Per ora lo ignoriamo, ma potremmo implementare una coda separata
            }
        }
    }
    
    private JsonObject sendCommandAndWaitResponse(JsonObject command) throws Exception {
        if (!connected) {
            throw new IllegalStateException("Non connesso al robot");
        }
        
        // Pulisci coda messaggi
        responseQueue.clear();
        
        // Invia comando
        String message = gson.toJson(command);
        client.send(message);
        logger.debug("📤 Inviato: {}", message);
        
        // Aspetta risposta per max 10 secondi
        JsonObject response = responseQueue.poll(10, TimeUnit.SECONDS);
        
        if (response == null) {
            throw new RuntimeException("Timeout: nessuna risposta dal server");
        }
        
        return response;
    }
    
    // === GETTERS ===
    
    public boolean isConnected() {
        return connected;
    }
    
    public boolean isEngaged() {
        return engaged;
    }
}
