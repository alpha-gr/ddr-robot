package main.java;

/**
 * Esempio di utilizzo del client sincrono semplificato
 */
public class RobotClientTest {
    
    public static void main(String[] args) {
        RobotClient robot = new RobotClient("ws://localhost:8765");
        
        try {
            // 1. Connessione
            System.out.println("🔌 Connessione al robot...");
            robot.connect();
            
            // 2. Attivazione
            System.out.println("🔗 Attivazione controllo remoto...");
            robot.engage();
            
            // 3. Movimento completo - ogni comando aspetta il completamento
            System.out.println("🎯 Test movimenti con attesa completamento...");
            
            System.out.println("\n--- Movimento 1 ---");
            String id1 = robot.moveRobot(0,0);
            System.out.println("✅ Movimento 1 COMPLETATO: " + id1);
            
            System.out.println("\n--- Movimento 2 ---");
            String id2 = robot.moveRobot(20, 60 );
            System.out.println("✅ Movimento 2 COMPLETATO: " + id2);
            
            System.out.println("\n--- Movimento 3 ---");
            String id3 = robot.moveRobot(50,50 );
            System.out.println("✅ Movimento 3 COMPLETATO: " + id3);
 
            
            // 5. Disattivazione
            System.out.println("\n🔗 Disattivazione controllo remoto...");
            robot.disengage();
            
        } catch (Exception e) {
            System.err.println("❌ Errore: " + e.getMessage());
            e.printStackTrace();
        } finally {
            robot.disconnect();
            System.out.println("👋 Disconnesso");
        }
    }
    
}
