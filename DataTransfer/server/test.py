import serial
import struct
import time
import math
import os
import socket

import registers
import utils

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server.bind(('0.0.0.0', 4444))
server.listen(2)


# =======================
#  OPEN SERIAL PORT
# =======================
ser = serial.Serial(
        '/dev/ttyACM0',   # change if your device is different
        baudrate=115200,
        timeout=1
        )

dataFile = "data.txt"
if os.path.exists(dataFile):
    os.remove(dataFile)
else: 
    f = open(dataFile, "x")

client, addr0 = server.accept()

print("Opened:", ser.name)
# =======================
#  CONFIGURE SWEEP
# =======================

start_freq       = 500_000      # 1 MHz
step_freq        = 50_000          # 1 kHz
requested_points = 201

print("Configuring sweep registers...")
registers.write_reg_8(0x00, start_freq, ser)
registers.write_reg_8(0x10, step_freq, ser)
registers.write_reg_2(0x20, requested_points, ser)

dev_points_bytes = registers.read_reg_2(0x20, ser)
if len(dev_points_bytes) != 2:
    raise RuntimeError("Failed to read sweepPoints back from device")

num_points = int.from_bytes(dev_points_bytes, 'little')
print(f"Requested sweepPoints = {requested_points}, device uses = {num_points}")

# =======================
#  CLEAR FIFO & START
# =======================

print("Clearing FIFO...")
registers.write_reg_1(0x30, 0, ser)
time.sleep(0.2)

print("Starting sweep...")
registers.write_reg_1(0x27, 1, ser)
time.sleep(0.2)

# =======================
#  READ FIFO
# =======================

raw = registers.read_fifo(0x30, num_points, ser)
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
    # conn.send(str(S11_mag).encode())
    S11_phase_deg = math.degrees(math.atan2(S11.imag, S11.real))

    # Return loss in dB
    S11_db = -20 * math.log10(S11_mag) if S11_mag > 0 else -999


    # Store
    freq = start_freq + i * step_freq
    freqs.append(freq)
    S11_mags.append(S11_mag)
    S11_phases.append(S11_phase_deg)
    S11_dB.append(S11_db)

    data = [freq, S11]

    utils.writeFile(dataFile, data)
    # send clean formatted line

    msg = f"{data[0]},{data[1].real},{data[1].imag}\n"
    client.send(msg.encode())

    print("Sent:", msg.strip())

print("\nDone.\n")
