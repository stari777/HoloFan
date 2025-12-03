from nicegui import ui, events
from PIL import Image
import numpy as np
import math
import io
import base64
import serial
import serial.tools.list_ports
import time
import asyncio

SIZE_XY = 30
BAUD = 115200
TIMEOUT = 1
MAX_WAIT = 60

current_image = None
preview_image = None
selected_port = None

# ==========================
# LOG-KONSOLE
# ==========================
log_output = ui.textarea(label='Event Log').classes('w-96 h-64')

def log(msg):
    log_output.value += msg + "\n"


# ==========================
# BILD-VERARBEITUNG
# ==========================
def image_to_pixels(img):
    img = img.convert("RGB")
    img = img.resize((SIZE_XY, SIZE_XY))
    arr = np.array(img, dtype=np.uint8)

    pixels = []
    for y in range(arr.shape[0]):
        for x in range(arr.shape[1]):
            r, g, b = arr[y, x]
            pixels.append((x, y, r, g, b))
    return pixels


def pixels_to_polar_lines(pixels):
    lines = []
    for (x, y, r, g, b) in pixels:
        rad = math.sqrt(x**2 + y**2)
        phi = math.atan2(y, x) * (180 / math.pi)
        lines.append(f"{rad:0.02f},{phi:0.02f},{r},{g},{b}")
    return lines


# ==========================
# SERIELLE ÜBERTRAGUNG
# ==========================
def send_lines_blocking(lines, port):
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        log("Connection established.")
    except Exception as e:
        return f"Serial error: {e}"

    ser.reset_input_buffer()

    # READY CHECK
    t0 = time.time()
    ready = False
    log("Waiting for READY...")

    while time.time() - t0 < MAX_WAIT:
        line = ser.readline().decode(errors='ignore').strip()
        if line == "READY":
            ready = True
            break

    if not ready:
        ser.close()
        return "ERROR: No READY from STM32."

    log("READY received. Sending image data...")

    # Daten senden (KEIN read-line mehr → kein Timeout-Lag!)
    for i, l in enumerate(lines):
        ser.write(l.encode() + b'\n')
        time.sleep(0.0005)

    ser.close()
    return "OK"


# ==========================
# NICEGUI HANDLER
# ==========================
async def handle_upload(e: events.UploadEventArguments):
    global current_image

    try:
        data = await e.file.read()
        current_image = Image.open(io.BytesIO(data))
        log("Image loaded.")
    except Exception as ex:
        ui.notify(f"Error loading image: {ex}", color='negative')
        current_image = None
        return

    buf = io.BytesIO()
    current_image.save(buf, format='PNG')
    b64 = base64.b64encode(buf.getvalue()).decode('ascii')
    preview_image.set_source(f'data:image/png;base64,{b64}')


async def on_send_click():
    global selected_port

    if current_image is None:
        ui.notify("Bitte zuerst ein Bild hochladen!", color='warning')
        return

    if not selected_port:
        ui.notify("Bitte COM-Port auswählen!", color='warning')
        return

    log("Converting image...")

    pixels = image_to_pixels(current_image)
    lines = pixels_to_polar_lines(pixels)

    log("Image converted. Starting transmission...")

    result = await asyncio.to_thread(send_lines_blocking, lines, selected_port)

    if result == "OK":
        log("Image data sent successfully.")
        ui.notify("Done!")
    else:
        log(result)
        ui.notify("Transmission failed!", color='negative')


# ==========================
# COM PORT HANDLING
# ==========================
def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    dropdown.options = ports

    if ports:
        dropdown.value = ports[0]
        set_selected_port(ports[0])
        log("COM ports refreshed.")
    else:
        dropdown.value = None
        set_selected_port(None)
        log("No COM ports found.")


def set_selected_port(value):
    global selected_port
    selected_port = value


# ==========================
# UI LAYOUT
# ==========================
with ui.row():
    ui.upload(on_upload=handle_upload).props('accept=image/*').classes('w-64')
    preview_image = ui.image().classes('w-64 h-64 object-contain')

with ui.row():
    dropdown = ui.select(
        options=[],
        label="COM-Port auswählen",
        on_change=lambda e: set_selected_port(e.value)
    )
    ui.button("Refresh Ports", on_click=refresh_ports)

ui.button("Send Image (Nucleo L432KC)", on_click=on_send_click)

refresh_ports()
ui.run()
