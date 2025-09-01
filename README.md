# DDR Robot - Sistema di Controllo Integrato

Sistema di controllo in tempo reale per differential drive robot con feedback visivo tramite marker ArUco.

## 🚀 Quick Start

### 1. Setup Hardware
1. **Stampare i marker ArUco:**
   ```bash
   cd robotvision/markers
   python genera_marker.py
   ```
   - Stampa `aruco_0.png` → Attacca al robot
   - Stampa `aruco_1.png` - `aruco_4.png` → Posiziona agli angoli dell'arena

2. **Setup Raspberry Pi:**
   - Assicurati che `robotserver.py` sia in esecuzione sul Raspberry Pi
   - Annota l'IP del Raspberry Pi (es. 192.168.1.100)

### 2. Configurazione Software
1. **Installa dipendenze:**
   ```bash
   pip install opencv-python numpy websockets asyncio
   ```

2. **Configura IP Raspberry Pi:**
   Modifica `config.py`:
   ```python
   WS_HOST = "192.168.1.100"  # IP del tuo Raspberry Pi
   SIMULATION_MODE = False    # False per controllo reale
   ```

### 3. Esecuzione Sistema Completo
```bash
python integrated_robot_system.py
```

## 🎮 Controlli

### Interfaccia Utente
- **Click Mouse:** Imposta posizione target nell'arena
- **'c':** Toggle controllo automatico ON/OFF  
- **'s':** Stop di emergenza
- **'r':** Reset target corrente
- **'q':** Chiudi programma

### Indicatori Visivi
- **Cerchio Giallo:** Robot (freccia rossa = direzione)
- **Cerchi Colorati:** Marker arena (1=blu, 2=verde, 3=rosso, 4=magenta)
- **Cerchio Magenta:** Target impostato
- **Linee Bianche:** Perimetro arena

## 📁 Struttura Progetto

```
ddr-robot/
├── config.py                    # Configurazione centralizzata
├── robot_communication.py       # Comunicazione WebSocket
├── robot_controller.py         # Controllore PID e logica movimento  
├── integrated_robot_system.py  # Sistema completo integrato
├── DESIGN_DOCUMENT.md          # Documentazione tecnica
├── robotvision/
│   ├── markers/
│   │   ├── genera_marker.py    # Generatore marker ArUco
│   │   └── *.png              # Marker stampabili
│   └── test_single_marker.py   # Test sistema visione
└── robotcontrol/
    └── robotserver.py          # Server robot (Raspberry Pi)
```

## ⚙️ Configurazione Avanzata

### Parametri PID (config.py)
Regola questi valori per ottimizzare il comportamento:
```python
PID_LINEAR_KP = 0.5      # Risposta proporzionale posizione
PID_ANGULAR_KP = 1.0     # Risposta proporzionale rotazione
POSITION_TOLERANCE = 2.0 # Precisione arrivo target
```

### Limiti di Sicurezza
```python
MAX_LINEAR_SPEED = 0.8   # Velocità massima [0-1]
BOUNDARY_MARGIN = 5.0    # Margine dai bordi arena
EMERGENCY_STOP_DISTANCE = 3.0  # Distanza stop emergenza
```

## 🔧 Test e Debug

### Test Componenti Singoli

1. **Test Solo Visione:**
   ```bash
   cd robotvision
   python test_single_marker.py
   ```

2. **Test Solo Comunicazione:**
   ```bash
   python robot_communication.py
   ```

3. **Test Solo Controller:**
   ```bash
   python robot_controller.py
   ```

### Modalità Simulazione
Per testare senza hardware:
```python
# In config.py
SIMULATION_MODE = True
```

### Logging e Diagnostica
Il sistema salva automaticamente:
- **Log eventi:** Console con timestamp
- **Traiettorie:** File JSON con coordinate/timestamp
- **Screenshots:** Premere 's' durante esecuzione

## 🐛 Troubleshooting

### Problemi Comuni

**1. "Impossibile connettersi al robot"**
- Verifica IP Raspberry Pi in `config.py`
- Controlla che `robotserver.py` sia in esecuzione
- Testa connettività: `ping IP_RASPBERRY_PI`

**2. "Tracking perso/instabile"**
- Migliora illuminazione dell'arena
- Stampa marker più grandi
- Verifica che marker non siano riflessi/danneggiati

**3. "Robot non raggiunge target/oscilla"**
- Riduci parametri PID (KP, KI, KD)
- Aumenta `POSITION_TOLERANCE`
- Verifica calibrazione arena (4 marker visibili)

**4. "Movimento erratico/troppo veloce"**
- Riduci `MAX_LINEAR_SPEED` e `MAX_ANGULAR_SPEED`
- Aumenta margini di sicurezza (`BOUNDARY_MARGIN`)

### Debug Avanzato

**Abilita logging dettagliato:**
```python
# In config.py
LOG_LEVEL = "DEBUG"
DEBUG_VISUALIZATION = True
```

**Monitor performance:**
- FPS mostrato in tempo reale
- Latenza comandi nel log
- Stato PID nei messaggi debug

## 📊 Metriche di Performance

### Target di Performance
- **FPS Video:** ≥30 FPS
- **Latenza Controllo:** <100ms
- **Precisione:** ±2% dell'arena  
- **Tempo Assestamento:** <3 secondi

### Monitoraggio
Il sistema traccia automaticamente:
- Frame rate video
- Tempo risposta comandi
- Errore di posizionamento
- Percentuale successi movimento

## 🔄 Workflow di Sviluppo

### Processo Iterativo
1. **Test** nuova feature in simulazione
2. **Valida** con robot fisico
3. **Misura** performance e stabilità  
4. **Ottimizza** parametri se necessario
5. **Documenta** risultati e next steps

### Prossimi Sviluppi
- [ ] Path planning avanzato (evitamento ostacoli)
- [ ] Controllo multi-robot
- [ ] Interfaccia web remota
- [ ] Calibrazione automatica camera
- [ ] Integrazione sensori aggiuntivi

---

## 📞 Supporto

Per problemi o miglioramenti, consulta:
1. `DESIGN_DOCUMENT.md` per dettagli tecnici
2. Log di sistema per errori specifici  
3. File di configurazione per parametri

**Buon controllo robotico!** 🤖
