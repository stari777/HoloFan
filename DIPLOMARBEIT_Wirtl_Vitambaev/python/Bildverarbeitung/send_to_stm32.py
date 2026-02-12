import serial
import time

# -----------------------------
# Konfiguration
# -----------------------------
PORT = "COM5"       # COM-Port, an dem der STM32 hängt
BAUD = 115200       # Baudrate muss mit STM32 übereinstimmen
FILE_PATH = "CoordToPolar"  # Vorbereitete Bilddatei
TIMEOUT = 1         # Sekunden, wie lange readline auf Daten wartet
MAX_WAIT = 20        # Sekunden, maximal auf READY warten
# -----------------------------

# Serial initialisieren
ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
ser.reset_input_buffer()

# Handshakex
print(f"Warte auf Rückmeldung vom uC an {PORT}...")
t0 = time.time()
ready = False

while time.time() - t0 < MAX_WAIT: # time.time einfach die zeit - t0 startzeitpunkt nach 5 sec keine rückmeldung
    line = ser.readline().decode(errors='ignore').strip()  # Zeile vom uC lesen
    if line:  # nur wenn tatsächlich etwas empfangen wurde
        print("Nucleo:", line)
    if line == "READY":
        ready = True
        break

if not ready:
    print("Keine Rückmeldung sende trotzdem (kein Handshake).")
else:
    print("Rückmeldung empfangen -> starte Dateiübertragung.")
    with open(FILE_PATH, "r") as f:
        for line in f:
            line = line.strip()
            ser.write(line.encode() + b'\r\n')       # Zeile senden
            time.sleep(0.002)              # kleine Pause, um STM32 nicht zu überlasten
            print(line)
            resp = ser.readline().decode(errors='ignore').strip()
            if resp:
                print("STM32 sagt:", resp)

print("Dateiübertragung abgeschlossen.")
ser.close()