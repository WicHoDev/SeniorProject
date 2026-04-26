"""
app_sim.py
==========
SFCW GPR web server — SIMULATION MODE
GPR data is generated synthetically.
Robot movement commands go to ESP32 over USB serial (no WiFi needed).

Connections:
  RPi USB port 1  -->  ESP32  (motor commands, serial)
  No STM32 needed in simulation mode

Install:
    pip3 install flask flask-socketio eventlet numpy scipy pyserial --break-system-packages

Run:
    python3 app_sim.py

Open in browser:
    http://<rpi-ip>:5000
"""

import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO, emit
import numpy as np
import threading
import time
import serial
from pathlib import Path
from datetime import datetime

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gpr_sim_secret'
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='eventlet')

# =============================================================================
#  HARDWARE CONFIGURATION
# =============================================================================

# ESP32 motor driver — GPIO UART (hardware serial through RPi GPIO pins)
# Wiring:  RPi GPIO14 (TX) --> ESP32 RX
#          RPi GPIO15 (RX) <-- ESP32 TX
#          RPi GND         --- ESP32 GND
# Both RPi and ESP32 run at 3.3V — no level shifter needed
# Enable UART on RPi first: see README for raspi-config steps
ESP32_PORT = '/dev/ttyAMA0'   # GPIO UART port on RPi 5
ESP32_BAUD = 115200

# Scan rate — traces per second
SCAN_HZ = 8

# Save directory
SAVE_DIR = Path('./scans')
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# =============================================================================
#  GPR PARAMETERS
# =============================================================================
F1          = 800e6
F2          = 1.2e9
NF          = 401
C0          = 3e8
ER_DRY      = 4.0
ER_WET      = 10.0
ALPHA_DRY   = 1.0
ALPHA_WET   = 3.5
NT          = 2048
ZMAX        = 1.5
GAIN_EXP    = 2.0
DYN_RANGE   = 40
MUTE_DEPTH  = 0.15
TAPER_DEPTH = 0.10
NX_WINDOW   = 80

# Simulation scene
PIPE_X0     = 0.5
PIPE_DEPTH  = 0.91
PIPE_RADIUS = 0.06
LEAK_RADIUS = 0.18
GAMMA_PVC   = -0.12
A_PIPE      = 1.5
NOISE_LEVEL = 0.012
N_ROCKS     = 10
A_ROCK      = 0.08
GAMMA_ROCK  = 0.04

# =============================================================================
#  PRECOMPUTED VECTORS
# =============================================================================
f          = np.linspace(F1, F2, NF)
df         = (F2 - F1) / (NF - 1)
v          = C0 / np.sqrt(ER_DRY)
depth_full = v * np.arange(NT) * (1.0 / (df * NT)) / 2
idz        = depth_full <= ZMAX
depth_plot = depth_full[idz]
Nd         = int(idz.sum())

mute_idx = int(np.argmax(depth_plot >= MUTE_DEPTH))
tap_idx  = int(np.argmax(depth_plot >= TAPER_DEPTH))
taper    = np.ones(Nd)
if tap_idx > 0:
    taper[:tap_idx] = 0.5 * (1 - np.cos(np.pi * np.arange(tap_idx) / tap_idx))
gain          = depth_plot ** GAIN_EXP
gain[gain==0] = 1.0
win           = np.hanning(NF)

rng    = np.random.default_rng(42)
rock_x = rng.uniform(0, 1, N_ROCKS)
rock_z = rng.uniform(0.3, 1.2, N_ROCKS)

# =============================================================================
#  STATE
# =============================================================================
state_lock = threading.Lock()

state = {
        'scanning':        False,
        'paused':          False,
        'leak_strength':   0.6,
        'robot_speed':     0,
        'robot_turn':      0,
        'robot_connected': False,
        'n_traces':        0,
        'er_est':          ER_DRY,
        'halo_score':      0.0,
        'verdict':         'idle',
        'severity':        'none',
        'pipe_detected':   False,
        'scans':           [],
        }

bscan      = np.full((Nd, NX_WINDOW), -DYN_RANGE, dtype=float)
write_col  = 0
bg_buf     = []
bg         = np.zeros(NF, dtype=complex)
bscan_lock = threading.Lock()

# =============================================================================
#  ESP32 SERIAL MOTOR CONTROL
#
#  Packet sent to ESP32 over USB serial (5 bytes):
#    byte 0:   0xFF  sync byte 1
#    byte 1:   0xFE  sync byte 2
#    byte 2:   speed as unsigned byte, offset +100
#              (so -100 becomes 0, 0 becomes 100, +100 becomes 200)
#    byte 3:   turn  as unsigned byte, offset +100
#    byte 4:   checksum = (0xFF + 0xFE + speed_byte + turn_byte) & 0xFF
#
#  ESP32 firmware reads this packet and drives motors accordingly.
#  Speed and turn are -100 to 100.
# =============================================================================

_esp32_ser  = None
_esp32_lock = threading.Lock()


def esp32_connect():
    """
    Opens the serial connection to the ESP32.
    Called once at startup and again if connection drops.
    """
    global _esp32_ser
    try:
        _esp32_ser = serial.Serial(ESP32_PORT, ESP32_BAUD, timeout=0.1)
        with state_lock:
            state['robot_connected'] = True
        print(f"ESP32 connected on {ESP32_PORT} @ {ESP32_BAUD}")
    except serial.SerialException as e:
        print(f"ESP32 not found on {ESP32_PORT}: {e}")
        print("Robot controls will not work until ESP32 is connected.")
        with state_lock:
            state['robot_connected'] = False


def send_move(speed: int, turn: int):
    """
    Send a movement command to the ESP32 over serial.
    speed: -100 (full reverse) to 100 (full forward)
    turn:  -100 (full left)    to 100 (full right)
    """
    global _esp32_ser
    speed  = int(np.clip(speed, -100, 100))
    turn   = int(np.clip(turn,  -100, 100))
    spd_b  = (speed + 100) & 0xFF
    trn_b  = (turn  + 100) & 0xFF
    chk    = (0xFF + 0xFE + spd_b + trn_b) & 0xFF
    packet = bytes([0xFF, 0xFE, spd_b, trn_b, chk])

    with _esp32_lock:
        try:
            if _esp32_ser is None or not _esp32_ser.is_open:
                esp32_connect()
            if _esp32_ser and _esp32_ser.is_open:
                _esp32_ser.write(packet)
                with state_lock:
                    state['robot_speed']     = speed
                    state['robot_turn']      = turn
                    state['robot_connected'] = True
        except serial.SerialException as e:
            print(f"ESP32 write error: {e}")
            with state_lock:
                state['robot_connected'] = False
            _esp32_ser = None


def send_stop():
    send_move(0, 0)


def monitor_esp32():
    """
    Background thread that checks the ESP32 connection every 5 seconds.
    Tries to reconnect if disconnected.
    """
    while True:
        with _esp32_lock:
            connected = _esp32_ser is not None and _esp32_ser.is_open
        if not connected:
            print("ESP32 disconnected — attempting reconnect...")
            esp32_connect()
            with state_lock:
                socketio.emit('status', {
                    'robot_connected': state['robot_connected']
                    })
        time.sleep(5)


# =============================================================================
#  SYNTHETIC TRACE GENERATOR
# =============================================================================

def generate_trace(x_ant: float, leak_strength: float) -> np.ndarray:
    S   = np.zeros(NF, dtype=complex)
    dx  = abs(x_ant - PIPE_X0)
    wf  = leak_strength * np.exp(-(dx / LEAK_RADIUS)**2)
    er_eff    = ER_DRY + wf * (ER_WET - ER_DRY)
    alpha_eff = ALPHA_DRY + wf * (ALPHA_WET - ALPHA_DRY)
    v_eff     = C0 / np.sqrt(er_eff)
    R         = np.sqrt(dx**2 + PIPE_DEPTH**2)
    geom      = np.exp(-(dx / (1.5 * PIPE_RADIUS))**2)
    S        += A_PIPE * GAMMA_PVC * geom * np.exp(-alpha_eff*R) * np.exp(-1j*2*np.pi*f*(2*R/v_eff))

    if leak_strength > 0.05:
        for doff in [0.08, 0.14, 0.20]:
            wd  = PIPE_DEPTH + doff
            ws  = LEAK_RADIUS * (1 + leak_strength * 0.5)
            if dx < ws * 2:
                R_w = np.sqrt(dx**2 + wd**2)
                v_w = C0 / np.sqrt(er_eff * 0.7 + ER_DRY * 0.3)
                gw  = np.exp(-(dx/ws)**2) * leak_strength * 0.25
                S  += gw * np.exp(-1j*2*np.pi*f*(2*R_w/v_w))

    S += 0.6 * np.exp(-1j*2*np.pi*f*1e-9)

    for r in range(N_ROCKS):
        dxr   = abs(x_ant - rock_x[r])
        Rr    = np.sqrt(dxr**2 + rock_z[r]**2)
        geomr = np.exp(-(dxr / (2*0.03))**2)
        S    += A_ROCK*GAMMA_ROCK*geomr*np.exp(-ALPHA_DRY*Rr)*np.exp(-1j*2*np.pi*f*(2*Rr/v))

    S += (rng.standard_normal(NF) + 1j*rng.standard_normal(NF)) * NOISE_LEVEL
    return S


# =============================================================================
#  SIGNAL PROCESSING
# =============================================================================

def process_trace(s_freq: np.ndarray, bg_est: np.ndarray) -> np.ndarray:
    """
    Returns LINEAR envelope (not dB).
    dB conversion happens later using global bscan max for consistent scaling.
    """
    s      = (s_freq - bg_est) * win
    s_time = np.fft.ifft(s, n=NT)
    env    = np.abs(s_time[:Nd])
    env[:mute_idx] = 0
    env    = env * taper * gain
    return env


def estimate_er(bscan_lin: np.ndarray) -> float:
    """
    Estimates effective permittivity from linear envelope bscan.
    Finds hyperbola apex by locating peak depth in each column
    then takes the median over the centre columns.
    """
    if bscan_lin.shape[1] < 5:
        return ER_DRY
    # Only look at columns with meaningful signal (avoid empty columns)
    col_max    = bscan_lin.max(axis=0)
    valid      = col_max > col_max.max() * 0.05
    if valid.sum() < 3:
        return ER_DRY
    col_peaks  = np.argmax(bscan_lin[:, valid], axis=0)
    apex_idx   = int(np.median(col_peaks))
    apex_depth = depth_plot[min(apex_idx, Nd-1)]
    two_way    = apex_depth * 2 / v * np.sqrt(ER_DRY)
    er_est     = (C0 * two_way / (2 * PIPE_DEPTH))**2
    return float(np.clip(er_est, ER_DRY, ER_WET))


def classify(er_est: float, halo: float, detected: bool):
    if not detected:                    return 'no pipe',       'none'
    if er_est < 4.6 and halo < 0.15:   return 'dry',           'none'
    elif er_est < 5.5 and halo < 0.35: return 'possible leak', 'possible'
    elif er_est < 7.0 or halo < 0.6:   return 'likely leak',   'likely'
    else:                               return 'major leak',    'major'


# =============================================================================
#  SCAN LOOP
# =============================================================================

def scan_loop():
    global bscan, write_col, bg_buf, bg

    ix          = 0
    x_positions = np.linspace(0, 1.0, NX_WINDOW)
    interval    = 1.0 / SCAN_HZ

    while True:
        with state_lock:
            scanning = state['scanning']
            paused   = state['paused']
            leak     = state['leak_strength']

        if not scanning or paused:
            time.sleep(0.05)
            continue

        t0     = time.perf_counter()
        x_ant  = x_positions[ix % NX_WINDOW]
        s_freq = generate_trace(x_ant, leak)
        ix    += 1

        bg_buf.append(s_freq)
        if len(bg_buf) > 15: bg_buf.pop(0)
        if len(bg_buf) >= 3:
            bg = np.mean(np.stack(bg_buf), axis=0)

        env = process_trace(s_freq, bg)

        with bscan_lock:
            # Store LINEAR envelope in buffer
            bscan[:, write_col] = env
            write_col = (write_col + 1) % NX_WINDOW

            with state_lock:
                state['n_traces'] += 1
                n = min(state['n_traces'], NX_WINDOW)

            if state['n_traces'] % 5 == 0 and n >= 5:
                # Metrics use linear values directly
                er_est = estimate_er(bscan)
                pi     = int(np.argmin(np.abs(depth_plot - PIPE_DEPTH)))
                bs_    = min(pi + 5, Nd-1)
                be_    = min(pi + int(0.3 / (ZMAX / Nd)), Nd)
                pipe_e = bscan[max(0, pi-3):pi+4, :].mean()
                halo   = (bscan[bs_:be_,:].mean() / (pipe_e+1e-10) if be_ > bs_ else 0.0)
                det    = pipe_e > 1e-6 and n >= 10
                verdict, severity = classify(er_est, halo, det)
                with state_lock:
                    state['er_est']        = round(float(er_est), 2)
                    state['halo_score']    = round(float(halo), 3)
                    state['verdict']       = verdict
                    state['severity']      = severity
                    state['pipe_detected'] = det

            # Convert to dB using global max across entire buffer for consistent scaling
            display   = np.roll(bscan, -write_col, axis=1)
            glob_max  = display.max() + 1e-300
            disp_dB   = 20 * np.log10(display / glob_max + 1e-300)
            # Clip to dynamic range
            disp_dB   = np.clip(disp_dB, -DYN_RANGE, 0)
            ds_idx    = np.linspace(0, Nd-1, 100).astype(int)
            ds_bscan  = disp_dB[ds_idx, :].tolist()

        with state_lock:
            payload = {
                    'bscan':           ds_bscan,
                    'depth_axis':      depth_plot[ds_idx].tolist(),
                    'trace_idx':       state['n_traces'],
                    'er':              state['er_est'],
                    'halo':            state['halo_score'],
                    'verdict':         state['verdict'],
                    'severity':        state['severity'],
                    'pipe_detected':   state['pipe_detected'],
                    'leak_strength':   state['leak_strength'],
                    'robot_connected': state['robot_connected'],
                    }

        socketio.emit('gpr_update', payload)
        time.sleep(max(0, interval - (time.perf_counter() - t0)))


# =============================================================================
#  FLASK ROUTES
# =============================================================================

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    with state_lock:
        return jsonify({k: v for k, v in state.items() if k != 'scans'})


@app.route('/api/scans')
def api_scans():
    with state_lock:
        return jsonify(state['scans'])


# =============================================================================
#  WEBSOCKET EVENTS
# =============================================================================

@socketio.on('connect')
def on_connect():
    with state_lock:
        s = dict(state)
    emit('status', s)


@socketio.on('cmd_move')
def on_move(data):
    speed = int(data.get('speed', 0))
    turn  = int(data.get('turn',  0))
    send_move(speed, turn)
    with state_lock:
        s = {
                'robot_speed':     state['robot_speed'],
                'robot_turn':      state['robot_turn'],
                'robot_connected': state['robot_connected'],
                }
    socketio.emit('status', s)


@socketio.on('cmd_stop')
def on_stop():
    send_stop()
    with state_lock:
        state['robot_speed'] = 0
        state['robot_turn']  = 0
    socketio.emit('status', {'robot_speed': 0, 'robot_turn': 0})


@socketio.on('cmd_scan_start')
def on_scan_start():
    with state_lock:
        state['scanning'] = True
        state['paused']   = False
    socketio.emit('status', {'scanning': True, 'paused': False})


@socketio.on('cmd_scan_stop')
def on_scan_stop():
    with state_lock:
        state['scanning'] = False
    socketio.emit('status', {'scanning': False})


@socketio.on('cmd_scan_pause')
def on_scan_pause():
    with state_lock:
        state['paused'] = not state['paused']
        p = state['paused']
    socketio.emit('status', {'paused': p})


@socketio.on('cmd_set_leak')
def on_set_leak(data):
    with state_lock:
        state['leak_strength'] = float(data.get('value', 0.6))


@socketio.on('cmd_save_scan')
def on_save_scan():
    ts   = datetime.now().strftime('%Y%m%d_%H%M%S')
    path = SAVE_DIR / f'scan_sim_{ts}.npz'
    with bscan_lock:
        b = bscan.copy()
    with state_lock:
        er  = state['er_est']
        vrd = state['verdict']
    np.savez(str(path), bscan=b, depth_plot=depth_plot, er_est=er, verdict=vrd)
    entry = {'id': ts, 'filename': path.name,
             'verdict': vrd, 'er': er, 'time': ts}
    with state_lock:
        state['scans'].append(entry)
    socketio.emit('scan_saved', entry)


@socketio.on('cmd_source')
def on_source(data):
    with state_lock:
        state['source'] = data.get('source', 'sim')
    socketio.emit('status', {'source': state.get('source', 'sim')})


# =============================================================================
#  STARTUP
# =============================================================================

if __name__ == '__main__':
    esp32_connect()
    threading.Thread(target=scan_loop,     daemon=True).start()
    threading.Thread(target=monitor_esp32, daemon=True).start()

    print("=" * 50)
    print("GPR Web Server — SIMULATION MODE")
    print(f"  Website:  http://0.0.0.0:5000")
    print(f"  ESP32:    {ESP32_PORT} @ {ESP32_BAUD} (USB serial)")
    print(f"  Scan Hz:  {SCAN_HZ}")
    print(f"  Saves to: {SAVE_DIR}")
    print("  Open browser: http://<rpi-ip>:5000")
    print("=" * 50)

    socketio.run(app, host='0.0.0.0', port=5000, debug=False)

