package main.java;

import com.google.gson.JsonObject;

public class CommandResponse {
    private final String status;
    private final String message;
    private final String movementId;
    private final JsonObject rawResponse;
    
    public CommandResponse(String status, String message, String movementId, JsonObject rawResponse) {
        this.status = status;
        this.message = message;
        this.movementId = movementId;
        this.rawResponse = rawResponse;
    }
    
    public static CommandResponse fromJson(JsonObject json) {
        String status = json.has("status") ? json.get("status").getAsString() : "unknown";
        String message = json.has("message") ? json.get("message").getAsString() : "";
        String movementId = json.has("movement_id") ? json.get("movement_id").getAsString() : null;
        
        return new CommandResponse(status, message, movementId, json);
    }
    
    public String getStatus() { return status; }
    public String getMessage() { return message; }
    public String getMovementId() { return movementId; }
    public JsonObject getRawResponse() { return rawResponse; }
    
    public boolean isSuccess() {
        return "success".equals(status);
    }
    
    @Override
    public String toString() {
        return String.format("CommandResponse{status='%s', message='%s', movementId='%s'}", 
                           status, message, movementId);
    }
}