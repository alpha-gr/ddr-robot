# Differential Drive Robot a Controllo Remoto

## Descrizione
Questo progetto realizza un **Differential Drive Robot (DDR) fisico** integrato con il sistema *cargoservice*,
originariamente sviluppato per l’esame di Ingegneria dei Sistemi Software.  
L’obiettivo è sostituire il Virtual Robot utilizzato in fase di sviluppo con un robot reale, mantenendo la compatibilità con il modello di comunicazione esistente.

## Relazione di progetto
La relazione completa del progetto è disponibile [qui](./userDocs/relazione.pdf).

## Architettura del sistema
Il sistema si compone di tre macro-componenti software principali:

1. **Sistema di controllo locale**  
   - Eseguito a bordo del robot (Raspberry Pi).  
   - Riceve comandi via WebSocket e gestisce l’attuazione dei motori tramite L298.  
   - Supporta comandi di movimento base (`forward`, `backward`, `left`, `right`, `stop`) con controllo PWM.

2. **Sistema di controllo centrale**  
   - Coordina e supervisiona il robot.  
   - Monitora posizione e orientamento tramite **fiducial markers ArUco** e una telecamera overhead.  
   - Implementa pathfinding (A* con euristica octile distance) e controllo PID per movimenti precisi.

3. **Attore QAK**  
   - Middleware di integrazione con *cargoservice*.  
   - Traduce comandi ad alto livello (`engage`, `moverobot`, `pickUpCargo`, …) in messaggi WebSocket JSON.  
   - Mantiene la compatibilità con il protocollo usato dal modulo `basicrobot`.

## Comunicazione
- **Trasporto:** WebSocket  
- **Formato messaggi:** JSON  
- **Esempi di comandi inviati dal client al server:**
  ```json
  { "command": "engage" }
  { "command": "disengage" }
  { "command": "movetosquare", "x": 1, "y": 2 }
  ```
- **Risposte del server:**
  ```json
  { "status": "success", "message": "..." }
  { "status": "error", "message": "descrizione" }
  ```
- **Evento asincrono di completamento movimento:**
  ```json
  { "command": "moverobotdone", "movement_id": "id", "status": "success" }
  ```

## Hardware
- Telaio robotico con due motori DC + ruota pivotante.  
- Driver motori **L298**.  
- **Raspberry Pi 3B** per controllo e networking.  
- Powerbank USB per alimentazione Raspberry Pi.  
- Telecamera overhead per rilevamento marker ArUco.

---

## 🏗️ Architettura del Sistema

```
┌─────────────────┐    WebSocket    ┌──────────────────┐
│   PC Controllo  │◄──────────────► │   Raspberry Pi   │
│                 │                 │                  │
│ ┌─────────────┐ │                 │ ┌──────────────┐ │
│ │Vision System│ │                 │ │Motor Control │ │
│ │   (ArUco)   │ │                 │ │   (L298N)    │ │
│ └─────────────┘ │                 │ └──────────────┘ │
│ ┌─────────────┐ │                 │                  │
│ │ Pathfinding │ │    Commands     │     ┌─────┐      │
│ │    (A*)     │ │ ──────────────► │     │Robot│      │
│ └─────────────┘ │                 │     └─────┘      │
│ ┌─────────────┐ │                 │                  │
│ │ PID Control │ │                 │                  │
│ └─────────────┘ │                 │                  │
│ ┌─────────────┐ │                 │                  │
│ │     UI      │ │                 │                  │
│ └─────────────┘ │                 │                  │
└─────────────────┘                 └──────────────────┘
```
