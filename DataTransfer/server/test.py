import serial
import struct
import time
import math

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
    # 0x18 = READFIFO
    ser.write(bytes([0x18, addr, count]))
    return ser.read(count * 32)   # 32 bytes per point

# =======================
#  CONFIGURE SWEEP
# =======================

# What we *ask* for (device may clamp this)
start_freq      = 1_000_000      # 1 MHz
step_freq       = 1_000          # 1 kHz
requested_points = 101           # we want 101, but device may override

print("Configuring sweep registers...")
write_reg_8(0x00, start_freq)        # sweepStartHz
write_reg_8(0x10, step_freq)         # sweepStepHz
write_reg_2(0x20, requested_points)  # sweepPoints (device may change this)

# Read back what the device actually accepted
dev_points_bytes = read_reg_2(0x20)
if len(dev_points_bytes) != 2:
    raise RuntimeError("Failed to read sweepPoints back from device")

dev_points = int.from_bytes(dev_points_bytes, 'little')
print(f"Requested sweepPoints = {requested_points}, device uses = {dev_points}")

# Use the device's number of points from now on
num_points = dev_points

# =======================
#  CLEAR FIFO, START SWEEP
# =======================

print("Clearing FIFO...")
write_reg_1(0x30, 0)          # any value clears FIFO
time.sleep(0.05)

print("Starting sweep...")
write_reg_1(0x27, 1)          # trigger sweep
time.sleep(0.2)               # give it time to finish (tune if needed)

# =======================
#  READ FIFO
# =======================

raw = read_fifo(0x30, num_points)
print("RAW LENGTH:", len(raw), "bytes (expected", num_points * 32, ")")

expected_len = num_points * 32
if len(raw) != expected_len:
    print("\n[WARNING]")
    print(f"Device returned {len(raw)} bytes, expected {expected_len}.")
    print("We will parse only full 32-byte blocks that exist.\n")

# =======================
#  PARSE BLOCKS
# =======================

max_blocks = len(raw) // 32   # number of full blocks we actually have
print(f"Parsing {max_blocks} full blocks...\n")

for i in range(max_blocks):
    block = raw[i*32:(i+1)*32]

    if len(block) != 32:
        print(f"[ERROR] Block {i} length = {len(block)}")
        break

    # Extract raw IQ components
    fwd0Re = struct.unpack('<i', block[0:4])[0]
    fwd0Im = struct.unpack('<i', block[4:8])[0]
    rev0Re = struct.unpack('<i', block[8:12])[0]
    rev0Im = struct.unpack('<i', block[12:16])[0]
    rev1Re = struct.unpack('<i', block[16:20])[0]
    rev1Im = struct.unpack('<i', block[20:24])[0]
    freqIdx = struct.unpack('<H', block[24:26])[0]

    # Compute S11 and S21 complex values
    # S11 = reflected / forward
    fwd_complex = complex(fwd0Re, fwd0Im)
    refl_complex = complex(rev0Re, rev0Im)
    thru_complex = complex(rev1Re, rev1Im)

    if fwd_complex != 0:
        S11 = refl_complex / fwd_complex
        S21 = thru_complex / fwd_complex
    else:
        S11 = 0
        S21 = 0

    # Magnitude and phase
    S11_mag = abs(S11)

    print(f"Point {i:3d} (freqIdx={freqIdx:3d}): "
          f"S11 = {S11.real:.4e} + j{S11.imag:.4e}, ")



print("\nDone.")

