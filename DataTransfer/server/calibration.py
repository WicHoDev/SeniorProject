import serial
import struct
import time

PORT = "/dev/ttyACM0"
BAUDRATE = 115200
POINTS = 201
START_FREQ = 500_000_000
STOP_FREQ = 3_000_000_000
STEP_FREQ = int((STOP_FREQ - START_FREQ) / (POINTS - 1))

def write_reg_8(ser, addr, value):
    ser.write(bytes([0x23, addr]) + value.to_bytes(8, 'little'))

def write_reg_2(ser, addr, value):
    ser.write(bytes([0x21, addr, value & 0xFF, (value >> 8) & 0xFF]))

def trigger_sweep(ser):
    ser.write(bytes([0x27, 1]))

def flush_fifo(ser):
    ser.write(bytes([0x30, 0]))
    time.sleep(0.05)
    ser.write(bytes([0x30, 0]))
    time.sleep(0.05)

def read_sweep(ser, points):
    ser.write(bytes([0x18, 0x30, points]))
    data = ser.read(32 * points)
    return data

def parse_sweep(data, points):
    results = []
    for i in range(points):
        block = data[i*32:(i+1)*32]
        if len(block) < 32:
            continue
        rev_re = struct.unpack('<i', block[8:12])[0]
        rev_im = struct.unpack('<i', block[12:16])[0]
        fwd_re = struct.unpack('<i', block[0:4])[0]
        fwd_im = struct.unpack('<i', block[4:8])[0]
        freq_index = struct.unpack('<H', block[24:26])[0]
        freq = START_FREQ + freq_index * STEP_FREQ
        fwd = complex(fwd_re, fwd_im)
        rev = complex(rev_re, rev_im)
        s11 = rev / fwd if fwd != 0 else 0
        results.append((freq / 1e6, s11))
    return sorted(results, key=lambda x: x[0])

def sweep_and_print(label):
    input(f"Connect {label} to CH0, then press ENTER...")
    flush_fifo(ser)
    trigger_sweep(ser)
    time.sleep(0.4)
    data = read_sweep(ser, POINTS)
    samples = parse_sweep(data, POINTS)
    print(f"\n[{label}] Sweep Results: {len(samples)} samples (sorted)")
    for f, s in samples:
        print(f"{f:9.3f} MHz | {s.real:+.3e}{s.imag:+.3e}j")
    return samples

with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
    write_reg_8(ser, 0x00, START_FREQ)
    write_reg_8(ser, 0x10, STEP_FREQ)
    write_reg_2(ser, 0x20, POINTS)

    load_data = sweep_and_print("LOAD")
    open_data = sweep_and_print("OPEN")
    short_data = sweep_and_print("SHORT")

