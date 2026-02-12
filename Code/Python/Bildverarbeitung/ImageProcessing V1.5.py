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

# ==========================
# SETTINGS
# ==========================
SIZE_XY = 30
BAUD = 115200
TIMEOUT = 1
MAX_WAIT = 60
LOG_WIDTH = 260

current_image = None
preview_image = None
selected_port = None
dark_mode = True


# ==========================
# GLOBAL DARK/LIGHT THEME CSS
# ==========================
DARK_CSS = """
body { background: #0e0e0e !important; color: white !important; }
textarea, .q-field { background: rgba(255,255,255,0.05) !important; color: white !important; }
"""

LIGHT_CSS = """
body { background: #f0f0f0 !important; color: black !important; }
textarea, .q-field { background: white !important; color: black !important; }
"""

def apply_theme():
    ui.add_head_html(f"<style>{DARK_CSS if dark_mode else LIGHT_CSS}</style>")

apply_theme()


# ==========================
# GLOW BUTTON STYLES
# ==========================
ui.add_head_html("""
<style>
.glow-button {
    transition: all 0.25s ease !important;
    box-shadow: 0 0 0 rgba(0,0,0,0) !important;
}
.glow-button:hover {
    transform: scale(1.05) !important;
    box-shadow: 0 0 25px rgba(0,128,255,0.7) !important;
}
.big-button {
    padding: 18px 32px !important;
    font-size: 1.1rem !important;
}
</style>
""")


# ==========================
# AUTO-SCROLL SCRIPT (läuft im Browser)
# ==========================
# Dieses Script prüft regelmäßig, ob ein <textarea> existiert und scrollt es ans Ende.
# Läuft clientseitig, deshalb löst es keine Fehler beim App-Start aus.
ui.add_head_html("""
<script>
window.addEventListener('load', function () {
    // Intervall: alle 200 ms prüfen und falls vorhanden ans Ende scrollen
    setInterval(function () {
        try {
            var ta = document.querySelector('textarea');
            if (ta) {
                ta.scrollTop = ta.scrollHeight;
            }
        } catch (e) {
            // still safe — ignore
        }
    }, 200);
});
</script>
""")


# ==========================
# DARK MODE SWITCH
# ==========================
def toggle_dark_mode():
    global dark_mode
    dark_mode = not dark_mode
    apply_theme()
    log(f"Theme switched to: {'dark' if dark_mode else 'light'}")


# ==========================
# LOG PANEL (LEFT FIXED)
# ==========================
with ui.column().style(
    f'''
    position:fixed;
    left:0;
    top:0;
    height:100vh;
    width:{LOG_WIDTH}px;
    padding:20px;
    box-sizing:border-box;

    backdrop-filter: blur(10px);
    background: rgba(255,255,255,0.07);
    border-right: 1px solid rgba(255,255,255,0.15);
    box-shadow: 3px 0 20px rgba(0,0,0,0.25);
    '''
):
    ui.label("Event Log").classes('text-xl font-bold mb-3')
    log_output = ui.textarea().classes('w-full').style(
        'height:82vh; border-radius:10px; padding:12px; resize:none; overflow-y: auto;'
    )


# ==========================
# LOG FUNCTION (einfach, ohne JS-Aufrufe hier)
# ==========================
def log(msg: str):
    # Füge Zeitstempel hinzu (optional). Wenn du es nicht willst, entferne das next line prefix.
    ts = time.strftime('%H:%M:%S')
    log_output.value += f"[{ts}] {msg}\n"


# ==========================
# IMAGE PROCESSING
# ==========================
def image_to_pixels(img):
    img = img.convert("RGB")
    img = img.resize((SIZE_XY, SIZE_XY))
    arr = np.array(img, dtype=np.uint8)
    return [(x, y, *arr[y, x]) for y in range(SIZE_XY) for x in range(SIZE_XY)]


def pixels_to_polar_lines(pixels):
    return [
        f"{math.sqrt(x**2 + y**2):0.02f},{math.degrees(math.atan2(y, x)):0.02f},{r},{g},{b}"
        for (x, y, r, g, b) in pixels
    ]


# ==========================
# SERIAL TX
# ==========================
def send_lines_blocking(lines, port):
    try:
        ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
        log("Connection established.")
    except Exception as e:
        return f"Serial error: {e}"

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    log("Waiting for READY...")

    t0 = time.time()
    ready = False
    while time.time() - t0 < MAX_WAIT:
        try:
            line = ser.readline().decode(errors='ignore').strip()
        except Exception:
            line = ""
        if line == "READY":
            ready = True
            break

    if not ready:
        try:
            ser.close()
        except Exception:
            pass
        return "ERROR: No READY from MCU."

    log("READY received. Sending image data...")

    for l in lines:
        try:
            ser.write(l.encode() + b'\n')
        except Exception as e:
            log(f"Write error: {e}")
            break
        time.sleep(0.0005)

    try:
        ser.close()
    except Exception:
        pass
    return "OK"


# ==========================
# HANDLERS
# ==========================
async def handle_upload(e: events.UploadEventArguments):
    global current_image

    try:
        data = await e.file.read()
        current_image = Image.open(io.BytesIO(data))
        log("Image loaded.")
    except Exception as ex:
        ui.notify(f"Error loading image: {ex}", color='negative')
        return

    buf = io.BytesIO()
    current_image.save(buf, format='PNG')
    preview_image.set_source(
        f'data:image/png;base64,{base64.b64encode(buf.getvalue()).decode()}'
    )


async def on_send_click():
    if current_image is None:
        ui.notify("Upload an image first!", color='warning')
        return
    if not selected_port:
        ui.notify("Select COM port!", color='warning')
        return

    log("Converting image...")
    lines = pixels_to_polar_lines(image_to_pixels(current_image))
    log("Converted. Sending...")

    result = await asyncio.to_thread(send_lines_blocking, lines, selected_port)
    if result == "OK":
        log("Image sent successfully!")
        ui.notify("Done!")
    else:
        log(result)
        ui.notify("Transmission failed!", color='negative')


def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    dropdown.options = ports
    dropdown.value = ports[0] if ports else None
    set_selected_port(dropdown.value)
    log("Ports refreshed.")


def set_selected_port(v):
    global selected_port
    selected_port = v


def set_baudrate(v):
    global BAUD
    try:
        BAUD = int(v)
        log(f"Baudrate set to {BAUD}")
    except:
        log("Invalid baudrate!")


# ==========================
# MAIN UI
# ==========================
with ui.column().style(
    f'''
    margin-left:{LOG_WIDTH}px;
    height:100vh;
    display:flex;
    justify-content:center;
    align-items:center;
    '''
):
    with ui.column().style(
        '''
        padding:40px;
        border-radius:20px;
        width:720px;

        background: rgba(255,255,255,0.10);
        backdrop-filter: blur(15px);
        box-shadow: 0 8px 40px rgba(0,0,0,0.3);
        '''
    ).classes('items-center'):

        with ui.row().style('width:100%; justify-content:flex-start; margin-bottom:12px;'):
            ui.switch("Dark Mode", value=True, on_change=lambda _: toggle_dark_mode())

        ui.separator().style('width:100%; margin-bottom:20px;')

        with ui.row().style('margin-top:15px; gap:30px; width:100%; justify-content:center;'):
            ui.upload(on_upload=handle_upload).props('accept=image/*').classes('w-72')
            preview_image = ui.image().classes('w-72 h-72 object-contain rounded-xl shadow-lg')

        with ui.row().style('margin-top:25px; gap:20px; width:100%; justify-content:center;'):
            dropdown = ui.select(options=[], label="COM-Port auswählen",
                                 on_change=lambda e: set_selected_port(e.value)).classes('w-40')

            ui.input("Baudrate", value=str(BAUD),
                     on_change=lambda e: set_baudrate(e.value)).classes('w-40')

            ui.button("REFRESH PORTS", on_click=refresh_ports).classes(
                'glow-button big-button bg-blue-500 text-white rounded-lg shadow'
            )

        ui.button("SEND IMAGE (NUCLEO L432KC)", on_click=on_send_click).classes(
            'glow-button big-button bg-gradient-to-r from-blue-500 to-indigo-600 '
            'text-white mt-6 rounded-lg shadow-xl font-bold'
        )


# INIT
refresh_ports()
ui.run()