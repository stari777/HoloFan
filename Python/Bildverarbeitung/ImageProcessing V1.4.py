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

current_image = None
preview_image = None
selected_port = None
dark_mode = True


# ==========================
# GLOBAL DARK/LIGHT CSS
# ==========================
DARK_CSS = """
body {
    background: #0f0f0f !important;
    color: white !important;
}
textarea, .q-field {
    background: rgba(255,255,255,0.05) !important;
    color: white !important;
}
"""

LIGHT_CSS = """
body {
    background: #f0f0f0 !important;
    color: black !important;
}
textarea, .q-field {
    background: white !important;
    color: black !important;
}
"""


def apply_theme():
    """Apply CSS theme based on current mode."""
    ui.add_head_html(
        f"<style>{DARK_CSS if dark_mode else LIGHT_CSS}</style>"
    )


def toggle_dark_mode():
    global dark_mode
    dark_mode = not dark_mode
    apply_theme()
    log(f"Theme switched to: {'dark' if dark_mode else 'light'}")


# INIT CSS
apply_theme()


# ==========================
# LOG PANEL LEFT (SEXY)
# ==========================
with ui.column().style(
    '''
    position:fixed;
    left:0;
    top:0;
    height:100vh;
    width:260px;
    padding:20px;
    box-sizing:border-box;

    backdrop-filter: blur(10px);
    background: rgba(255,255,255,0.07);
    border-right: 1px solid rgba(255,255,255,0.15);
    box-shadow: 3px 0 20px rgba(0,0,0,0.25);
    '''
):

    ui.label("Event Log").classes('text-xl font-bold mb-3')
    log_output = ui.textarea().classes('w-full').style('height:82vh; border-radius:10px;')

def log(msg: str):
    log_output.value += msg + "\n"


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

    ser.reset_input_buffer()

    log("Waiting for READY...")
    t0 = time.time()
    ready = False

    while time.time() - t0 < MAX_WAIT:
        if ser.readline().decode(errors='ignore').strip() == "READY":
            ready = True
            break

    if not ready:
        ser.close()
        return "ERROR: No READY from MCU."

    log("READY received. Sending...")

    for l in lines:
        ser.write(l.encode() + b'\n')
        time.sleep(0.0005)

    ser.close()
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
        ui.notify(f"Load error: {ex}")
        return

    buf = io.BytesIO()
    current_image.save(buf, format='PNG')
    preview_image.set_source(f'data:image/png;base64,{base64.b64encode(buf.getvalue()).decode()}')


async def on_send_click():
    if current_image is None:
        ui.notify("Upload image first!", color='warning')
        return
    if not selected_port:
        ui.notify("Select COM port!", color='warning')
        return

    log("Converting...")
    lines = pixels_to_polar_lines(image_to_pixels(current_image))
    log("Converted. Sending...")

    result = await asyncio.to_thread(send_lines_blocking, lines, selected_port)
    if result == "OK":
        log("Done!")
        ui.notify("Sent!")
    else:
        log(result)
        ui.notify("Error!", color='negative')


def refresh_ports():
    ports = [p.device for p in serial.tools.list_ports.comports()]
    dropdown.options = ports
    dropdown.value = ports[0] if ports else None
    set_selected_port(dropdown.value)
    log("Ports refreshed.")

def set_selected_port(v):
    global selected_port
    selected_port = v


# ==========================
# CENTERED SEXY UI — NOW PURE CSS (NO JS)
# ==========================
with ui.column().classes(
    'items-center justify-center'
).style(
    f'''
    margin-left:{260}px;
    height:100vh;
    display:flex;
    justify-content:center;
    align-items:center;
    '''
):

    with ui.column().style(
        '''
        padding:30px;
        border-radius:20px;
        width:460px;

        background: rgba(255,255,255,0.10);
        backdrop-filter: blur(15px);
        box-shadow: 0 8px 40px rgba(0,0,0,0.3);

        transition: all 0.3s;
        '''
    ).classes('items-center'):

        ui.switch("Dark Mode", value=True, on_change=lambda _: toggle_dark_mode())

        ui.separator()

        with ui.row().style('margin-top:25px; gap:20px;'):
            ui.upload(on_upload=handle_upload).props('accept=image/*').classes('w-52')
            preview_image = ui.image().classes('w-52 h-52 object-contain rounded-xl shadow-lg')

        with ui.row().style('margin-top:25px; gap:12px;'):
            dropdown = ui.select(options=[], label="COM-Port",
                                 on_change=lambda e: set_selected_port(e.value))
            ui.button("REFRESH PORTS", on_click=refresh_ports).classes(
                'bg-blue-500 text-white rounded-lg px-4 py-2 shadow hover:bg-blue-600 transition-all'
            )

        ui.button("SEND IMAGE (NUCLEO L432KC)", on_click=on_send_click).classes(
            'bg-gradient-to-r from-blue-500 to-indigo-600 text-white '
            'px-6 py-3 rounded-lg shadow-xl font-bold '
            'hover:scale-105 hover:shadow-2xl transition-all mt-4'
        )
# INIT
refresh_ports()
ui.run()
