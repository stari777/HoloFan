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
LOG_WIDTH = 280

current_image = None
selected_port = None
dark_mode = True
is_sending = False


# ==========================
# GLOBAL DARK/LIGHT THEME CSS
# ==========================
DARK_CSS = """
body { 
    background: linear-gradient(135deg, #0f0f23 0%, #1a0a2e 100%) !important; 
    color: white !important; 
}
textarea, .q-field { 
    background: rgba(255,255,255,0.08) !important; 
    color: white !important;
    border: 1px solid rgba(255,255,255,0.1) !important;
}
"""

LIGHT_CSS = """
body { 
    background: linear-gradient(135deg, #f5f5f5 0%, #e8e8f0 100%) !important; 
    color: #333 !important; 
}
textarea, .q-field { 
    background: rgba(255,255,255,0.95) !important; 
    color: #333 !important;
    border: 1px solid rgba(0,0,0,0.1) !important;
}

.q-field__native, .q-field__input {
    color: #333 !important;
}

.q-field__label {
    color: #333 !important;
}

.q-menu {
    background: rgba(255,255,255,0.95) !important;
    border: 1px solid rgba(0,0,0,0.1) !important;
}

.q-item {
    color: #333 !important;
    background: rgba(255,255,255,0.95) !important;
}

.q-item:hover {
    background: rgba(100,150,255,0.15) !important;
}

.q-item__label {
    color: #333 !important;
}

.q-menu .q-item {
    color: #333 !important;
    background: rgba(255,255,255,0.95) !important;
}

.q-menu .q-item:hover {
    background: rgba(100,150,255,0.15) !important;
    color: #333 !important;
}

.q-menu .q-item__label {
    color: #333 !important;
}
"""

DARK_CSS_EXTENDED = """
.q-field__native, .q-field__input {
    color: white !important;
}

.q-field__label {
    color: white !important;
}

.q-menu {
    background: rgba(20, 20, 60, 0.95) !important;
    color: white !important;
}

.q-item {
    color: white !important;
}

.q-item__label {
    color: white !important;
}

.q-menu .q-item {
    color: white !important;
    background: rgba(20, 20, 60, 0.9) !important;
}

.q-menu .q-item:hover {
    background: rgba(100, 150, 255, 0.3) !important;
    color: white !important;
}

.q-menu .q-item__label {
    color: white !important;
}
"""

def apply_theme():
    css = DARK_CSS + DARK_CSS_EXTENDED if dark_mode else LIGHT_CSS
    ui.add_head_html(f"<style data-theme-style='true'>{css}</style>")

# Add an additional override stylesheet that will always be applied
ui.add_head_html("""
<style id="dropdown-override">
/* This will be updated dynamically */
</style>
""")

apply_theme()


# ==========================
# ENHANCED BUTTON STYLES & ANIMATIONS
# ==========================
ui.add_head_html("""
<style>
.glow-button {
    transition: all 0.3s ease !important;
    position: relative;
    overflow: hidden;
}

.glow-button::before {
    content: '';
    position: absolute;
    top: 0;
    left: -100%;
    width: 100%;
    height: 100%;
    background: rgba(255,255,255,0.3);
    transition: left 0.5s ease !important;
}

.glow-button:hover::before {
    left: 100%;
}

.glow-button:hover {
    transform: translateY(-2px) !important;
    box-shadow: 0 0 30px rgba(100,200,255,0.6), 0 5px 20px rgba(100,200,255,0.3) !important;
}

.glow-button:active {
    transform: translateY(0) !important;
}

.refresh-button {
    padding: 12px 20px !important;
    font-size: 0.95rem !important;
}

.send-button {
    padding: 18px 40px !important;
    font-size: 1.15rem !important;
    letter-spacing: 1px;
}

.send-button:disabled {
    opacity: 0.5 !important;
    cursor: not-allowed !important;
}



</style>
""")


# ==========================
# AUTO-SCROLL SCRIPT
# ==========================
ui.add_head_html("""
<script>
window.addEventListener('load', function () {
    setInterval(function () {
        try {
            var ta = document.querySelector('textarea');
            if (ta) {
                ta.scrollTop = ta.scrollHeight;
            }
        } catch (e) {
            // silent
        }
    }, 200);
});
</script>
""")


# ==========================
# DARK MODE SWITCH
# ==========================
def on_dark_mode_change(e):
    global dark_mode
    dark_mode = e.value
    
    # Build CSS
    if dark_mode:
        css = DARK_CSS + DARK_CSS_EXTENDED
        dropdown_override = ".q-menu .q-item { color: white !important; background: rgba(20, 20, 60, 0.9) !important; } .q-menu .q-item:hover { background: rgba(100, 150, 255, 0.3) !important; } .q-menu .q-item__label { color: white !important; } .q-menu { background: rgba(20, 20, 60, 0.95) !important; }"
    else:
        css = LIGHT_CSS
        dropdown_override = ".q-menu .q-item { color: #333 !important; background: rgba(255, 255, 255, 0.95) !important; } .q-menu .q-item:hover { background: rgba(100, 150, 255, 0.15) !important; } .q-menu .q-item__label { color: #333 !important; } .q-menu { background: rgba(255, 255, 255, 0.95) !important; }"
    
    # Update main CSS
    main_update = f"""
    <script>
        setTimeout(function() {{
            var oldStyles = document.querySelectorAll('[data-theme-style]');
            oldStyles.forEach(function(style) {{
                style.remove();
            }});
        }}, 5);
    </script>
    <style data-theme-style="true">
    {css}
    </style>
    """
    
    # Update dropdown override CSS
    override_update = f"""
    <script>
        setTimeout(function() {{
            var override = document.getElementById('dropdown-override');
            if (override) {{
                override.textContent = '{dropdown_override}';
            }}
        }}, 10);
    </script>
    """
    
    ui.add_head_html(main_update + override_update)
    
    theme_name = "dark" if dark_mode else "light"
    log(f"Theme switched to: {theme_name}")


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

    backdrop-filter: blur(15px);
    background: rgba(255,255,255,0.08);
    border-right: 2px solid rgba(100,200,255,0.2);
    box-shadow: 4px 0 30px rgba(0,0,0,0.3);
    '''
):
    ui.label("Event Log").classes('text-xl font-bold mb-3')
    log_output = ui.textarea().classes('w-full').style(
        'height: calc(100vh - 100px); border-radius:12px; padding:12px; resize:none; overflow-y: auto; font-size: 0.9em; color: white; flex: 1;'
    )


# ==========================
# LOG FUNCTION
# ==========================
def log(msg: str):
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
    lines = []
    for (x, y, r, g, b) in pixels:
        # Radius und Winkel berechnen
        rad = math.sqrt(x**2 + y**2)
        phi = math.degrees(math.atan2(y, x))
        
        # Als Integer * 100 speichern (wie im STM32 Script)
        rad_int = int(rad * 100)
        phi_int = int(phi * 100)
        
        # RGB Werte als Integer
        r_int = int(r)
        g_int = int(g)
        b_int = int(b)
        
        # Zeile zusammenbauen
        line = f"{rad_int},{phi_int},{r_int},{g_int},{b_int}"
        lines.append(line)
    
    return lines


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

    log("Waiting for READY from MCU...")

    t0 = time.time()
    ready = False
    while time.time() - t0 < MAX_WAIT:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                log(f"MCU: {line}")
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

    log("READY received. Starting transmission...")
    time.sleep(0.1)

    for i, l in enumerate(lines):
        try:
            ser.write(l.encode() + b'\n')
        except Exception as e:
            log(f"Write error: {e}")
            break
        time.sleep(0.005)
        
        # Progress logging
        if (i + 1) % 100 == 0:
            progress = int((i + 1) / len(lines) * 100)
            log(f"Progress: {progress}%")

    log("Transmission completed.")
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
        log(f"Image loaded ({current_image.size[0]}x{current_image.size[1]})")
    except Exception as ex:
        ui.notify(f"Error loading image: {ex}", color='negative')
        return
    
    log("Image ready for transmission")
    
    send_btn.enabled = True


async def on_send_click():
    global is_sending
    
    if current_image is None:
        ui.notify("Upload an image first!", color='warning')
        return
    if not selected_port:
        ui.notify("Select COM port!", color='warning')
        return

    is_sending = True
    send_btn.enabled = False
    
    log("Converting image to polar coordinates...")
    lines = pixels_to_polar_lines(image_to_pixels(current_image))
    log(f"Converted. {len(lines)} pixels ready.")
    log("Starting transmission...")

    result = await asyncio.to_thread(send_lines_blocking, lines, selected_port)
    
    is_sending = False
    
    if result == "OK":
        log("Image sent successfully!")
        ui.notify("Transmission complete!", color='positive')
    else:
        log(f"Error: {result}")
        ui.notify("Transmission failed!", color='negative')
    
    send_btn.enabled = True


def refresh_ports():
    global selected_port
    ports = [p.device for p in serial.tools.list_ports.comports()]
    dropdown.options = ports
    if ports:
        dropdown.value = ports[0]
        selected_port = ports[0]
        log(f"Found {len(ports)} COM port(s)")
    else:
        log("No COM ports found")
        dropdown.value = None
        selected_port = None


def set_selected_port(v):
    global selected_port
    selected_port = v
    if v:
        log(f"Selected port: {v}")


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
    padding:40px;
    '''
):
    with ui.column().style(
        '''
        gap:30px;
        align-items:center;
        '''
    ):
        # HEADER
        with ui.row().style('width:100%; justify-content:space-between; align-items:center;'):
            ui.label("HOLOGRAM FAN UPLOADER").classes('text-3xl font-bold')
            dark_mode_switch = ui.switch("Dark Mode", value=dark_mode, on_change=on_dark_mode_change).classes('text-lg')

        ui.separator().style('width:100%; height:2px; background: rgba(100,200,255,0.2);')

        # MAIN CONTENT
        with ui.row().style('gap:50px; width:100%; justify-content:center; align-items:flex-start;'):
            
            # LEFT: Upload
            with ui.column().style('gap:20px; align-items:center;'):
                ui.label("Select Image").classes('text-xl font-bold')
                ui.upload(on_upload=handle_upload).props('accept=image/*').classes('w-80').style(
                    'border: 2px dashed rgba(100,200,255,0.4); border-radius: 12px; padding: 20px;'
                )
            

            # RIGHT: Controls
            with ui.column().style('gap:20px; min-width:320px; position: relative; z-index: 1;'):
                ui.label("Configuration").classes('text-xl font-bold')
                
                dropdown = ui.select(
                    options=[], 
                    label="COM Port",
                    on_change=lambda e: set_selected_port(e.value)
                ).classes('w-full').style('border-radius: 8px; z-index: 10;')
                
                baudrate_select = ui.select(
                    options=['9600', '19200', '38400', '57600', '115200'],
                    value='115200',
                    label="Baudrate",
                    on_change=lambda e: set_baudrate(e.value)
                ).classes('w-full').style('border-radius: 8px;')
                baudrate_select.classes(add='!z-50')
                
                ui.button("REFRESH PORTS", on_click=refresh_ports).classes(
                    'glow-button refresh-button w-full bg-blue-500 text-white rounded-lg'
                )
                
                ui.separator().style('width:100%; height:1px; background: rgba(100,200,255,0.2);')
                
                ui.label("Transmission Settings").classes('text-sm font-semibold opacity-75')
                ui.label(f"Image Size: 30x30 pixels").classes('text-sm opacity-60')
                ui.label(f"Data Points: 900").classes('text-sm opacity-60')
                ui.label(f"Format: Polar (distance, angle, RGB)").classes('text-sm opacity-60')
                
                ui.separator().style('width:100%; height:1px; background: rgba(100,200,255,0.2);')
                
                send_btn = ui.button("SEND TO HOLOGRAM", on_click=on_send_click).classes(
                    'glow-button send-button w-full bg-gradient-to-r from-cyan-500 to-blue-600 text-white rounded-lg'
                )
                send_btn.enabled = False


# INIT
log("Application started")
log("Upload an image to begin")
refresh_ports()
ui.run()