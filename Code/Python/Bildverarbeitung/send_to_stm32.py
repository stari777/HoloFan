import serial
import time
import os

# -----------------------------
# Konfiguration
# -----------------------------
PORT = "COM4"       # COM-Port, an dem der STM32 hängt
BAUD = 115200       # Baudrate muss mit STM32 übereinstimmen
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
FILE_PATH = os.path.join(SCRIPT_DIR, "CoordToPolar")
TIMEOUT = 1         # Sekunden, wie lange readline auf Daten wartet 
MAX_WAIT = 1        # Sekunden, maximal auf READY warten
# -----------------------------

# Serial initialisieren
ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
ser.reset_input_buffer()

# Handshake
print(f"Warte auf Rückmeldung vom uC an {PORT}...")
t0 = time.time()
ready = True

while time.time() - t0 < MAX_WAIT: # time.time vergangene Zeit - t0 Startzeitpunkt
    line = ser.readline().decode(errors='ignore').strip()  # Zeile vom µC lesen
    if line:  # nur wenn tatsächlich etwas empfangen wurde
        print("Nucleo:", line)
    if line == "READY":
        ready = True
        break

if not ready:
    print("Keine Rückmeldung")
else:
    print("Rückmeldung empfangen -> starte Dateiübertragung.")
    time.sleep(0.1) # kurze Pause
    with open(FILE_PATH, "r") as f:
        for line in f:
            line = line.strip()
            time.sleep(0.005)               # kleine Pause, um STM32 nicht zu überlasten
            ser.write(line.encode() + b'\n')       # Zeile senden
            print(line)

            #resp = ser.readline().decode(errors='ignore').strip()
            #if resp:
            #   print("STM32:", resp)

print("Dateiübertragung abgeschlossen.")
ser.close()