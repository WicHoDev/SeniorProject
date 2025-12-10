import serial
import socket
import struct
import time
import math
import matplotlib.pyplot as plt

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 4444))
server.listen(2)
conn, addr1 = server.accept()
# data = conn.recv(1024).decode()


# =======================
#  OPEN SERIAL PORT
# =======================
ser = serial.Serial(
        '/dev/ttyACM0',   # change if your device is different
        baudrate=115200,
        timeout=1
        )

print("Opened:", ser.name)

# =======================
#  REGISTER HELPERS
# =======================

def write_reg_1(addr, val):
    ser.write(bytes([0x20, addr, val & 0xFF]))

def write_reg_2(addr, val):
    ser.write(bytes([
        0x21, addr,
        val & 0xFF,
        (val >> 8) & 0xFF
        ]))

def write_reg_4(addr, val):
    ser.write(bytes([0x22, addr]) + val.to_bytes(4, 'little'))

def write_reg_8(addr, val):
    ser.write(bytes([0x23, addr]) + val.to_bytes(8, 'little'))

def read_reg_1(addr):
    ser.write(bytes([0x10, addr]))
    return ser.read(1)

def read_reg_2(addr):
    ser.write(bytes([0x11, addr]))
    return ser.read(2)

def read_reg_4(addr):
    ser.write(bytes([0x12, addr]))
    return ser.read(4)

def read_fifo(addr, count):
    ser.write(bytes([0x18, addr, count]))
    return ser.read(count * 32)   # 32 bytes per point

# =======================
#  CONFIGURE SWEEP
# =======================

start_freq      = 1_000_000      # 1 MHz
step_freq       = 1_000          # 1 kHz
requested_points = 101

print("Configuring sweep registers...")
write_reg_8(0x00, start_freq)
write_reg_8(0x10, step_freq)
write_reg_2(0x20, requested_points)

dev_points_bytes = read_reg_2(0x20)
if len(dev_points_bytes) != 2:
    raise RuntimeError("Failed to read sweepPoints back from device")

num_points = int.from_bytes(dev_points_bytes, 'little')
print(f"Requested sweepPoints = {requested_points}, device uses = {num_points}")

# =======================
#  CLEAR FIFO & START
# =======================

print("Clearing FIFO...")
write_reg_1(0x30, 0)
time.sleep(0.05)

print("Starting sweep...")
write_reg_1(0x27, 1)
time.sleep(0.2)

# =======================
#  READ FIFO
# =======================

raw = read_fifo(0x30, num_points)
print("RAW LENGTH:", len(raw), "bytes (expected", num_points * 32, ")")

max_blocks = len(raw) // 32
print(f"Parsing {max_blocks} full blocks...\n")

# ------------------------
#   DATA STORAGE ARRAYS
# ------------------------
freqs = []
S11_mags = []
S11_phases = []
S11_dB = []
SWR_list = []

# =======================
#  PARSE BLOCKS
# =======================

for i in range(max_blocks):
    block = raw[i*32:(i+1)*32]

    fwd0Re = struct.unpack('<i', block[0:4])[0]
    fwd0Im = struct.unpack('<i', block[4:8])[0]
    rev0Re = struct.unpack('<i', block[8:12])[0]
    rev0Im = struct.unpack('<i', block[12:16])[0]

    fwd_complex = complex(fwd0Re, fwd0Im)
    refl_complex = complex(rev0Re, rev0Im)

    if fwd_complex != 0:
        S11 = refl_complex / fwd_complex
    else:
        S11 = 0

    # Magnitude & phase
    S11_mag = abs(S11)
    conn.send(str(S11_mag).encode())
    S11_phase_deg = math.degrees(math.atan2(S11.imag, S11.real))

    # Return loss in dB
    S11_db = -20 * math.log10(S11_mag) if S11_mag > 0 else -999

    # SWR
    if S11_mag >= 1:
        SWR = float("inf")
    else:
        SWR = (1 + S11_mag) / (1 - S11_mag)

    # Store
    freq = start_freq + i * step_freq
    freqs.append(freq)
    S11_mags.append(S11_mag)
    S11_phases.append(S11_phase_deg)
    S11_dB.append(S11_db)
    SWR_list.append(SWR)

    print(f"Point {i:3d}: freq={freq/1e6:.3f} MHz, "
          f"S11={S11.real:.4e}+j{S11.imag:.4e}, "
          f"SWR={SWR:.3f}, RL={S11_db:.2f} dB")

print("\nDone.\n")

# =======================
#  PLOTS
# =======================

# ---- Plot |S11| magnitude ----
plt.figure(figsize=(10,5))
plt.plot(freqs, S11_mags)
plt.title("S11 Magnitude vs Frequency")
plt.xlabel("Frequency (Hz)")
plt.ylabel("|S11| (linear)")
plt.grid(True)
plt.show()

# ---- Plot S11 return loss (dB) ----
plt.figure(figsize=(10,5))
plt.plot(freqs, S11_dB)
plt.title("S11 Return Loss (dB)")
plt.xlabel("Frequency (Hz)")
plt.ylabel("S11 (dB)")
plt.grid(True)
plt.show()
