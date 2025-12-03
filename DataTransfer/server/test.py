#!/usr/bin/env python3
import serial
import struct
import json
import time
import math

# ==========================
# USER SETTINGS
# ==========================
PORT = "/dev/ttyACM0"
CAL_FILE = "calibration.json"

SWEEP_DELAY = 0.40
MAX_RETRIES = 5

# ==========================
# SERIAL INIT
# ==========================
ser = serial.Serial(PORT, baudrate=115200, timeout=1)
print("Opened", ser.name)

# ==========================
# USB HELPERS
# ==========================
def write_reg_1(a, v): ser.write(bytes([0x20, a, v & 0xFF]))
def write_reg_2(a, v): ser.write(bytes([0x21, a, v & 0xFF, (v >> 8) & 0xFF]))
def write_reg_8(a, v): ser.write(bytes([0x23, a]) + v.to_bytes(8, "little"))
def read_reg_2(a): ser.write(bytes([0x11, a])); return ser.read(2)
def read_fifo(a, c): ser.write(bytes([0x18, a, c])); return ser.read(c * 32)

# ==========================
# LOAD CALIBRATION
# ==========================
def load_calibration(filename):
    with open(filename, "r") as f:
        d = json.load(f)

    start = d["start_freq"]
    step  = d["step_freq"]
    pts   = d["num_points"]

    E_d = [complex(x[0], x[1]) for x in d["E_d"]]
    E_s = [complex(x[0], x[1]) for x in d["E_s"]]
    E_r = [complex(x[0], x[1]) for x in d["E_r"]]

    print("\nLoaded calibration.")
    return start, step, pts, E_d, E_s, E_r

# ==========================
# PROGRAM SWEEP
# ==========================
def program_sweep(start, step, pts):
    write_reg_8(0x00, start)
    write_reg_8(0x10, step)
    write_reg_2(0x20, pts)

    dev_pts = int.from_bytes(read_reg_2(0x20), "little")
    print("Device points:", dev_pts)
    return dev_pts

# ==========================
# ROBUST SWEEP
# ==========================
def acquire_sweep(start_freq, step_freq, expected_points):

    for attempt in range(1, MAX_RETRIES + 1):
        print(f"Sweep attempt {attempt}/{MAX_RETRIES}")

        write_reg_1(0x30, 0); time.sleep(0.05)
        write_reg_1(0x30, 0); time.sleep(0.05)

        write_reg_1(0x27, 1)
        time.sleep(SWEEP_DELAY)

        raw = read_fifo(0x30, expected_points)
        n = len(raw) // 32
        print(" Points:", n)

        if n == expected_points:
            freqs = []
            S11 = []

            for i in range(n):
                blk = raw[i*32:(i+1)*32]

                fwdRe = struct.unpack("<i", blk[0:4])[0]
                fwdIm = struct.unpack("<i", blk[4:8])[0]
                revRe = struct.unpack("<i", blk[8:12])[0]
                revIm = struct.unpack("<i", blk[12:16])[0]

                fwd = complex(fwdRe, fwdIm)
                rev = complex(revRe, revIm)

                idx = struct.unpack("<H", blk[24:26])[0]
                freq = start_freq + idx * step_freq

                freqs.append(freq)
                S11.append(0+0j if fwd == 0 else rev / fwd)

            return freqs, S11

        print(" MISMATCH → retrying...")

    raise RuntimeError("Failed to acquire sweep.")

# ==========================
# APPLY CALIBRATION
# ==========================
def apply_calibration(s_raw, E_d, E_s, E_r):
    N = min(len(s_raw), len(E_d))
    out = []

    for i in range(N):
        m = s_raw[i]
        Ed, Es, Er = E_d[i], E_s[i], E_r[i]

        num = (m - Ed)
        den = Er + Es * (m - Ed)

        X = 0+0j if den == 0 else num / den
        out.append(X)

    return out

# ==========================
# MAIN
# ==========================
if __name__ == "__main__":
    start, step, pts, E_d, E_s, E_r = load_calibration(CAL_FILE)
    dev_pts = program_sweep(start, step, pts)

    input("\nConnect ANTENNA and press ENTER...")

    freqs, S11_raw = acquire_sweep(start, step, dev_pts)
    S11_cal = apply_calibration(S11_raw, E_d, E_s, E_r)

    print("\nFreq (MHz) | S11_cal (X+jY) |  dB  |  SWR | Error | |Error|")
    print("------------------------------------------------------------------------")

    for f, raw, cal in zip(freqs, S11_raw, S11_cal):
        freq_mhz = f / 1e6

        # SWR
        gamma = abs(cal)
        swr = (1+gamma)/(1-gamma) if gamma < 1 else float("inf")

        # dB magnitude
        if gamma == 0:
            db = float("-inf")
        else:
            db = 20 * math.log10(gamma)

        # Error
        err = raw - cal
        err_mag = abs(err)

        print(f"{freq_mhz:9.3f} | "
              f"{cal.real:+.3e}{cal.imag:+.3e}j | "
              f"{db:6.2f} | "
              f"{swr:6.2f} | "
              f"{err.real:+.3e}{err.imag:+.3e}j | "
              f"{err_mag:.3e}")

