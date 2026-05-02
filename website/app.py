#!/usr/bin/env python3
"""
motor_test.py
=============
Minimal interactive motor controller for the SFCW-GPR robot.

This bypasses the web server entirely. Just runs a command-line REPL
that lets you drive the motors directly via Pi GPIO → ESP32 → DRV8871.

Goal: prove that the Pi can talk to the ESP32 and the motors turn,
without any other moving parts in the system.

─────────────────────────────────────────────────────────────────
USAGE
─────────────────────────────────────────────────────────────────
    sudo apt install python3-gpiozero    # if not already installed
    python3 motor_test.py

Then type commands at the prompt:
    f       forward
    b       backward / reverse
    l       turn left  (left motor reverse, right motor forward)
    r       turn right (left motor forward, right motor reverse)
    s       stop  (coast)
    0..7    set speed level (0 = stop, 7 = full speed)
    t       run a self-test (cycles through every output combination)
    p       print current pin state
    q       quit (release pins)

─────────────────────────────────────────────────────────────────
WIRING (Pi 5 BCM → ESP32-S3 input)
─────────────────────────────────────────────────────────────────
    GPIO17 → IO_04   ENABLE
    GPIO27 → IO_05   DIR_L
    GPIO22 → IO_06   DIR_R
    GPIO23 → IO_07   SPD_S0
    GPIO24 → IO_08   SPD_S1
    GPIO25 → IO_15   SPD_S2
    GND    → GND     (mandatory — no shared ground = no signal)

Edit the GPIO_* constants below if your wiring is different.
"""

from gpiozero import DigitalOutputDevice
from time import sleep
import sys

# ─── Pi 5 GPIO PIN ASSIGNMENTS (BCM numbering) ──────────────────────
GPIO_EN     = 17
GPIO_DIR_L  = 27
GPIO_DIR_R  = 22
GPIO_SPD_S0 = 23
GPIO_SPD_S1 = 24
GPIO_SPD_S2 = 25

# ─── Default speed level (0–7) ──────────────────────────────────────
DEFAULT_SPEED = 4   # ≈57% PWM duty


# ════════════════════════════════════════════════════════════════════
#  GPIO SETUP
# ════════════════════════════════════════════════════════════════════
print("Opening GPIO pins...")
try:
    en  = DigitalOutputDevice(GPIO_EN,     active_high=True, initial_value=False)
    dl  = DigitalOutputDevice(GPIO_DIR_L,  active_high=True, initial_value=False)
    dr  = DigitalOutputDevice(GPIO_DIR_R,  active_high=True, initial_value=False)
    s0  = DigitalOutputDevice(GPIO_SPD_S0, active_high=True, initial_value=False)
    s1  = DigitalOutputDevice(GPIO_SPD_S1, active_high=True, initial_value=False)
    s2  = DigitalOutputDevice(GPIO_SPD_S2, active_high=True, initial_value=False)
except Exception as e:
    print(f"\n[ERROR] Could not open GPIO pins: {type(e).__name__}: {e}")
    print("\nTroubleshooting:")
    print("  • Are you on a Raspberry Pi? (this script only works on a Pi)")
    print("  • Is gpiozero installed? sudo apt install python3-gpiozero")
    print("  • Are the pins already in use? Try: sudo pkill -f python")
    print(f"  • Pin numbers requested: EN={GPIO_EN}, DL={GPIO_DIR_L}, DR={GPIO_DIR_R},")
    print(f"                           S0={GPIO_SPD_S0}, S1={GPIO_SPD_S1}, S2={GPIO_SPD_S2}")
    sys.exit(1)

print(f"  ENABLE = GPIO{GPIO_EN}")
print(f"  DIR_L  = GPIO{GPIO_DIR_L}")
print(f"  DIR_R  = GPIO{GPIO_DIR_R}")
print(f"  SPD_S0 = GPIO{GPIO_SPD_S0}")
print(f"  SPD_S1 = GPIO{GPIO_SPD_S1}")
print(f"  SPD_S2 = GPIO{GPIO_SPD_S2}")
print("GPIO ready.\n")


# ════════════════════════════════════════════════════════════════════
#  STATE
# ════════════════════════════════════════════════════════════════════
current_speed = DEFAULT_SPEED   # 0–7


# ════════════════════════════════════════════════════════════════════
#  LOW-LEVEL PIN WRITES
# ════════════════════════════════════════════════════════════════════
def write_pins(enable: int, dir_left: int, dir_right: int, speed: int):
    """
    Set all 6 control lines and print what was written.

    enable    : 0 (coast) or 1 (active)
    dir_left  : 0 (reverse) or 1 (forward)
    dir_right : 0 (reverse) or 1 (forward)
    speed     : 0–7 (3-bit speed level)
    """
    speed = max(0, min(7, speed))
    bit0 = (speed >> 0) & 1
    bit1 = (speed >> 1) & 1
    bit2 = (speed >> 2) & 1

    en.on()  if enable    else en.off()
    dl.on()  if dir_left  else dl.off()
    dr.on()  if dir_right else dr.off()
    s0.on()  if bit0      else s0.off()
    s1.on()  if bit1      else s1.off()
    s2.on()  if bit2      else s2.off()

    print(f"  → EN={enable}  DIR_L={dir_left}  DIR_R={dir_right}  "
          f"SPD={bit2}{bit1}{bit0} (level {speed})")


def print_state():
    print(f"  current pin state: "
          f"EN={int(en.value)}  DIR_L={int(dl.value)}  DIR_R={int(dr.value)}  "
          f"S2={int(s2.value)}  S1={int(s1.value)}  S0={int(s0.value)}")


# ════════════════════════════════════════════════════════════════════
#  HIGH-LEVEL MOTION COMMANDS
# ════════════════════════════════════════════════════════════════════
def forward():
    print(f"FORWARD at speed {current_speed}")
    write_pins(enable=1, dir_left=1, dir_right=1, speed=current_speed)


def backward():
    print(f"BACKWARD at speed {current_speed}")
    write_pins(enable=1, dir_left=0, dir_right=0, speed=current_speed)


def turn_left():
    print(f"TURN LEFT at speed {current_speed}")
    # Tank-style turn: left tracks back, right tracks forward
    write_pins(enable=1, dir_left=0, dir_right=1, speed=current_speed)


def turn_right():
    print(f"TURN RIGHT at speed {current_speed}")
    # Tank-style turn: left tracks forward, right tracks back
    write_pins(enable=1, dir_left=1, dir_right=0, speed=current_speed)


def stop():
    print("STOP (coast)")
    write_pins(enable=0, dir_left=0, dir_right=0, speed=0)


def set_speed(lvl: int):
    global current_speed
    current_speed = max(0, min(7, lvl))
    pct = [0, 14, 28, 43, 57, 71, 86, 100][current_speed]
    print(f"speed level → {current_speed} (~{pct}% PWM duty)")


# ════════════════════════════════════════════════════════════════════
#  SELF-TEST — runs through all 8 speed levels in both directions
# ════════════════════════════════════════════════════════════════════
def self_test():
    print("\n=== SELF-TEST ===")
    print("Stepping through all speed levels for 1 second each.")
    print("Watch the ESP32 status LED and listen to the motors.\n")

    print("[forward sweep]")
    for level in range(8):
        print(f"  level {level}:")
        write_pins(1, 1, 1, level)
        sleep(1.0)

    print("\n[backward sweep]")
    for level in range(8):
        print(f"  level {level}:")
        write_pins(1, 0, 0, level)
        sleep(1.0)

    print("\n[stop]")
    stop()
    print("=== SELF-TEST DONE ===\n")


# ════════════════════════════════════════════════════════════════════
#  COMMAND LOOP
# ════════════════════════════════════════════════════════════════════
HELP = """
Commands:
  f       forward
  b       backward
  l       turn left
  r       turn right
  s       stop (coast)
  0–7     set speed level
  t       self-test (cycles all speeds)
  p       print pin state
  h       this help
  q       quit
"""

print(HELP)

try:
    while True:
        cmd = input("motor> ").strip().lower()

        if   cmd == 'f':              forward()
        elif cmd == 'b':              backward()
        elif cmd == 'l':              turn_left()
        elif cmd == 'r':              turn_right()
        elif cmd == 's':              stop()
        elif cmd == 't':              self_test()
        elif cmd == 'p':              print_state()
        elif cmd == 'h':              print(HELP)
        elif cmd in ('q', 'quit', 'exit'): break
        elif cmd in ('0','1','2','3','4','5','6','7'):
            set_speed(int(cmd))
        elif cmd == '':
            continue
        else:
            print(f"unknown command: '{cmd}'  (type 'h' for help)")

except (KeyboardInterrupt, EOFError):
    print()  # newline after Ctrl+C

finally:
    print("Stopping motors and releasing pins...")
    try:
        en.off(); dl.off(); dr.off()
        s0.off(); s1.off(); s2.off()
        for pin in (en, dl, dr, s0, s1, s2):
            pin.close()
    except Exception as e:
        print(f"[WARN] cleanup error: {e}")
    print("Goodbye.")
