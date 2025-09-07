package main.java;

/**
 * Notifica di movimento completato
 * Semplificata per essere coerente con il protocollo WebSocket
 */
public class MovementCompletedNotification {
    private final String movementId;
    private final boolean success;
    private final String message;
    
    public MovementCompletedNotification(String movementId, boolean success, String message) {
        this.movementId = movementId;
        this.success = success;
        this.message = message;
    }
    
    // Getters
    public String getMovementId() { return movementId; }
    public boolean isSuccess() { return success; }
    public String getMessage() { return message; }
    
    @Override
    public String toString() {
        return String.format("MovementCompleted{id='%s', success=%s, message='%s'}",
                           movementId, success, message);
    }
}