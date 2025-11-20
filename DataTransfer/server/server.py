import serial
import struct
import time


# =======================
#  OPEN SERIAL PORT
# =======================
ser = serial.Serial(
        '/dev/ttyACM0',   # <-- change if needed
        baudrate=115200,
        timeout=1
        )

print("Opened:", ser.name)


# =======================
#  REGISTER WRITE HELPERS
# =======================

def write_reg_1(addr, val):
    ser.write(bytes([0x20, addr, val & 0xFF]))

def write_reg_2(addr, val):
    ser.write(bytes([0x21, addr,
                     val & 0xFF,
                     (val >> 8) & 0xFF]))

def write_reg_4(addr, val):
    ser.write(bytes([0x22, addr] + list(val.to_bytes(4, 'little'))))

def write_reg_8(addr, val):
    ser.write(bytes([0x23, addr] + list(val.to_bytes(8, 'little'))))


# =======================
#  REGISTER READ HELPERS
# =======================

def read_reg_1(addr):
    ser.write(bytes([0x10, addr]))
    return ser.read(1)

def read_reg_2(addr):
    ser.write(bytes([0x11, addr]))
    return ser.read(2)

def read_reg_4(addr):
    ser.write(bytes([0x12, addr]))
    return ser.read(4)


# =======================
#  FIFO READ
# =======================
def read_fifo(addr, count):
    # opcode = 0x18, address, count
    ser.write(bytes([0x18, addr, count]))
    return ser.read(count * 32)   # 32 bytes per sweep bin


# =====================================================
#              CONFIGURE SWEEP PARAMETERS
# =====================================================

start_freq = 1_000_000        # 1 MHz
step_freq  = 1_000            # 1 kHz
num_points = 101              # 101 sweep samples

print("Configuring sweep parameters...")

# Writing these registers automatically puts NanoVNA into USB mode
write_reg_8(0x00, start_freq)     # sweepStartHz
write_reg_8(0x10, step_freq)      # sweepStepHz
write_reg_2(0x20, num_points)     # sweepPoints


# =====================================================
#                 START SWEEP (IMPORTANT!)
# =====================================================

print("Starting sweep...")

write_reg_1(0x27, 1)   # TRIGGER SWEEP
time.sleep(0.15)       # MUST WAIT for sweep to complete


# =====================================================
#                 CLEAR FIFO
# =====================================================

print("Clearing FIFO...")
write_reg_1(0x30, 0)   # any value clears FIFO
time.sleep(0.05)


# =====================================================
#                 READ SWEEP DATA
# =====================================================

raw = read_fifo(0x30, num_points)

print("RAW LENGTH:", len(raw))  # EXPECT num_points * 32 (→ 3232 bytes)


# =====================================================
#            PARSE EACH 32-BYTE DATA BLOCK
# =====================================================

expected_len = num_points * 32

if len(raw) != expected_len:
    print("\n[WARNING]")
    print(f"Device returned {len(raw)} bytes, expected {expected_len}.")
    print("This means the device did NOT use your sweepPoints value.")
    print("Likely the VNA defaulted to 40 points.\n")


# Safe parsing loop
print("\nParsing blocks...\n")

for i in range(num_points):
    block = raw[i*32:(i+1)*32]

    # Safety check
    if len(block) != 32:
        print(f"[ERROR] Block {i} length = {len(block)} (expected 32)")
        break

    # Unpack frame according to Appendix II
    fwd0Re = struct.unpack('<i', block[0:4])[0]
    fwd0Im = struct.unpack('<i', block[4:8])[0]
    rev0Re = struct.unpack('<i', block[8:12])[0]
    rev0Im = struct.unpack('<i', block[12:16])[0]
    rev1Re = struct.unpack('<i', block[16:20])[0]
    rev1Im = struct.unpack('<i', block[20:24])[0]
    freqIdx = struct.unpack('<H', block[24:26])[0]

    # Example magnitude calculation
    mag = ((rev0Re**2 + rev0Im**2)**0.5) / ((fwd0Re**2 + fwd0Im**2)**0.5)

    print(f"Point {i}: freqIndex={freqIdx}, mag={mag:.4f}")


print("\nDone.")

