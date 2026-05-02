#!/usr/bin/env python3
"""
motor_control.py
================
Motor control library for the SFCW-GPR tank robot.

Drives 6 GPIO output pins from a Raspberry Pi 5 to an ESP32-S3 via
GPIO Zero (lgpio backend). The ESP32 receives the bit-bang signals
and generates PWM for two DRV8871-Q1 motor drivers.

CAN BE USED TWO WAYS:
─────────────────────
1) AS A MODULE
       from motor_control import MotorController

       mc = MotorController()
       mc.forward()
       mc.set_speed(5)
       mc.turn_left()
       mc.stop()
       mc.cleanup()

2) AS A STANDALONE TEST PROGRAM (interactive REPL)
       python3 motor_control.py

   Type 'h' at the prompt for a list of commands.

WIRING (Pi 5 BCM → ESP32-S3 input):
───────────────────────────────────
    GPIO17 → IO_04   ENABLE
    GPIO27 → IO_05   DIR_L
    GPIO22 → IO_06   DIR_R
    GPIO23 → IO_07   SPD_S0
    GPIO24 → IO_08   SPD_S1
    GPIO25 → IO_15   SPD_S2
    GND    → GND     (mandatory)

Edit the GPIO_* constants below if your Pi-side wiring differs.
"""

from gpiozero import DigitalOutputDevice
from time import sleep


# ════════════════════════════════════════════════════════════════════
#  PIN CONFIGURATION (Pi 5 BCM numbering)
# ════════════════════════════════════════════════════════════════════
GPIO_EN     = 17    # ENABLE
GPIO_DIR_L  = 27    # left motor direction
GPIO_DIR_R  = 22    # right motor direction
GPIO_SPD_S0 = 23    # speed bit 0 (LSB)
GPIO_SPD_S1 = 24    # speed bit 1
GPIO_SPD_S2 = 25    # speed bit 2 (MSB)


# ════════════════════════════════════════════════════════════════════
#  SPEED TABLE — index 0–7 maps to PWM duty percent
#  Must match the SPEED_TABLE in the ESP32 firmware (main.cpp)
# ════════════════════════════════════════════════════════════════════
SPEED_LEVELS = [0, 14, 28, 43, 57, 71, 86, 100]


# ════════════════════════════════════════════════════════════════════
#  MOTOR MODES
# ════════════════════════════════════════════════════════════════════
MODE_PWM      = 'pwm'        # variable speed via 3-bit speed code
MODE_FULL     = 'full'       # full speed only (speed code always 7)


# ════════════════════════════════════════════════════════════════════
#  MOTOR CONTROLLER
# ════════════════════════════════════════════════════════════════════
class MotorController:
    """
    Controls the tank robot motors via 6 GPIO output pins to the ESP32.

    Use as a context manager OR call cleanup() manually:

        with MotorController() as mc:
            mc.forward()
            sleep(2)
            mc.stop()

    Or:

        mc = MotorController()
        try:
            mc.forward()
            ...
        finally:
            mc.cleanup()
    """

    def __init__(self, default_speed: int = 4, mode: str = MODE_PWM,
                 verbose: bool = True):
        """
        default_speed : initial speed level 0–7 (default 4 = ~57% duty)
        mode          : MODE_PWM (variable speed) or MODE_FULL (always 100%)
        verbose       : print pin state on every write
        """
        self.verbose       = verbose
        self.mode          = mode
        self.current_speed = max(0, min(7, default_speed))
        self._pins         = {}

        if self.verbose:
            print("[motor] opening GPIO pins...")

        try:
            self._pins = {
                'en': DigitalOutputDevice(GPIO_EN,     active_high=True, initial_value=False),
                'dl': DigitalOutputDevice(GPIO_DIR_L,  active_high=True, initial_value=False),
                'dr': DigitalOutputDevice(GPIO_DIR_R,  active_high=True, initial_value=False),
                's0': DigitalOutputDevice(GPIO_SPD_S0, active_high=True, initial_value=False),
                's1': DigitalOutputDevice(GPIO_SPD_S1, active_high=True, initial_value=False),
                's2': DigitalOutputDevice(GPIO_SPD_S2, active_high=True, initial_value=False),
            }
        except Exception as e:
            print(f"[motor] ERROR opening pins: {type(e).__name__}: {e}")
            print("[motor]   • Is the lgpio backend installed?")
            print("[motor]       sudo apt install liblgpio-dev")
            print("[motor]       pip3 install lgpio")
            print("[motor]   • Are pins already in use?  pkill -f python")
            raise

        if self.verbose:
            print(f"[motor] ready  (mode={self.mode}, default speed={self.current_speed})")
            print(f"[motor] pin map: EN=GPIO{GPIO_EN}  DIR_L=GPIO{GPIO_DIR_L}  "
                  f"DIR_R=GPIO{GPIO_DIR_R}  S0/S1/S2=GPIO{GPIO_SPD_S0}/"
                  f"{GPIO_SPD_S1}/{GPIO_SPD_S2}")

    # ────────────────────────────────────────────────────────────────
    #  CONTEXT MANAGER SUPPORT
    # ────────────────────────────────────────────────────────────────
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        return False

    # ────────────────────────────────────────────────────────────────
    #  CONFIGURATION
    # ────────────────────────────────────────────────────────────────
    def set_mode(self, mode: str):
        """Switch between MODE_PWM (variable speed) and MODE_FULL (always 100%)."""
        if mode not in (MODE_PWM, MODE_FULL):
            raise ValueError(f"unknown mode '{mode}', use MODE_PWM or MODE_FULL")
        self.mode = mode
        if self.verbose:
            print(f"[motor] mode → {mode}")

    def set_speed(self, level: int):
        """Set PWM speed level (0–7). Has no effect when mode == MODE_FULL."""
        self.current_speed = max(0, min(7, level))
        if self.verbose:
            pct = SPEED_LEVELS[self.current_speed]
            tag = "(ignored, mode=FULL)" if self.mode == MODE_FULL else ""
            print(f"[motor] speed → level {self.current_speed} (~{pct}% duty) {tag}")

    # ────────────────────────────────────────────────────────────────
    #  LOW-LEVEL PIN WRITE
    # ────────────────────────────────────────────────────────────────
    def _write(self, enable: int, dir_l: int, dir_r: int, speed: int):
        """Write all 6 pins. speed is 0-7 (3-bit)."""
        if not self._pins:
            print("[motor] write skipped — controller not initialized")
            return

        speed = max(0, min(7, speed))
        s0 = (speed >> 0) & 1
        s1 = (speed >> 1) & 1
        s2 = (speed >> 2) & 1

        bits = {'en': enable, 'dl': dir_l, 'dr': dir_r,
                's0': s0, 's1': s1, 's2': s2}

        for name, val in bits.items():
            (self._pins[name].on if val else self._pins[name].off)()

        if self.verbose:
            print(f"[motor] EN={enable} DIR_L={dir_l} DIR_R={dir_r} "
                  f"SPD={s2}{s1}{s0} (level {speed})")

    def _effective_speed(self) -> int:
        """Returns the speed code that will actually be sent given current mode."""
        return 7 if self.mode == MODE_FULL else self.current_speed

    # ────────────────────────────────────────────────────────────────
    #  HIGH-LEVEL MOTION COMMANDS
    # ────────────────────────────────────────────────────────────────
    def forward(self):
        """Both motors forward."""
        self._write(enable=1, dir_l=1, dir_r=1, speed=self._effective_speed())

    def backward(self):
        """Both motors reverse."""
        self._write(enable=1, dir_l=0, dir_r=0, speed=self._effective_speed())

    def turn_left(self):
        """Tank turn: left tracks reverse, right tracks forward."""
        self._write(enable=1, dir_l=0, dir_r=1, speed=self._effective_speed())

    def turn_right(self):
        """Tank turn: left tracks forward, right tracks reverse."""
        self._write(enable=1, dir_l=1, dir_r=0, speed=self._effective_speed())

    def stop(self):
        """ENABLE LOW → both motors coast."""
        self._write(enable=0, dir_l=0, dir_r=0, speed=0)

    # ────────────────────────────────────────────────────────────────
    #  GENERIC INPUT (for the web server's tank-mix output)
    # ────────────────────────────────────────────────────────────────
    def drive(self, speed: int, turn: int):
        """
        Tank-drive mixing for speed/turn percentages.

        speed : -100 (reverse) to +100 (forward)
        turn  : -100 (left)    to +100 (right)
        """
        left  = max(-100, min(100, speed + turn))
        right = max(-100, min(100, speed - turn))

        if left == 0 and right == 0:
            self.stop()
            return

        dir_l = 1 if left  >= 0 else 0
        dir_r = 1 if right >= 0 else 0

        if self.mode == MODE_FULL:
            spd_idx = 7
        else:
            mag = max(abs(left), abs(right))
            spd_idx = min(7, round(mag * 7 / 100))

        self._write(enable=1, dir_l=dir_l, dir_r=dir_r, speed=spd_idx)

    # ────────────────────────────────────────────────────────────────
    #  STATUS
    # ────────────────────────────────────────────────────────────────
    def get_state(self) -> dict:
        """Return current pin state as a dict (for debug/UI display)."""
        if not self._pins:
            return {'available': False}
        return {
            'available': True,
            'mode':      self.mode,
            'speed':     self.current_speed,
            'en':        int(self._pins['en'].value),
            'dir_l':     int(self._pins['dl'].value),
            'dir_r':     int(self._pins['dr'].value),
            's0':        int(self._pins['s0'].value),
            's1':        int(self._pins['s1'].value),
            's2':        int(self._pins['s2'].value),
        }

    # ────────────────────────────────────────────────────────────────
    #  CLEANUP
    # ────────────────────────────────────────────────────────────────
    def cleanup(self):
        """Stop motors and release all pins."""
        if not self._pins:
            return
        if self.verbose:
            print("[motor] stopping and releasing pins...")
        try:
            for pin in self._pins.values():
                pin.off()
            for pin in self._pins.values():
                pin.close()
        except Exception as e:
            print(f"[motor] cleanup warning: {e}")
        self._pins = {}


# ════════════════════════════════════════════════════════════════════
#  STANDALONE TEST REPL — runs only when executed directly
# ════════════════════════════════════════════════════════════════════
HELP_TEXT = """
─── Motor Test Commands ─────────────────────────────
  f         forward
  b         backward
  l         turn left
  r         turn right
  s         stop (coast)
  0..7      set PWM speed level (PWM mode only)
  m pwm     switch to PWM mode (variable speed)
  m full    switch to FULL mode (always 100%)
  t         self-test (cycles all speeds and directions)
  p         print pin state
  h         show this help
  q         quit
─────────────────────────────────────────────────────
"""


def _self_test(mc: MotorController):
    """Cycle through all speed levels in both directions."""
    print("\n=== SELF-TEST ===")
    print("Stepping through speed levels (1 second each).")
    print("Watch the ESP32 LED and listen to the motors.\n")

    print("[forward sweep]")
    for level in range(8):
        mc.set_speed(level)
        mc.forward()
        sleep(1.0)

    print("\n[backward sweep]")
    for level in range(8):
        mc.set_speed(level)
        mc.backward()
        sleep(1.0)

    print("\n[stop]")
    mc.stop()
    print("=== DONE ===\n")


def main():
    """Interactive command-line motor test."""
    try:
        mc = MotorController(default_speed=4, mode=MODE_PWM, verbose=True)
    except Exception:
        return

    print(HELP_TEXT)

    try:
        while True:
            try:
                cmd = input("motor> ").strip().lower()
            except EOFError:
                break

            if not cmd:
                continue

            if   cmd == 'f':  mc.forward()
            elif cmd == 'b':  mc.backward()
            elif cmd == 'l':  mc.turn_left()
            elif cmd == 'r':  mc.turn_right()
            elif cmd == 's':  mc.stop()
            elif cmd == 't':  _self_test(mc)
            elif cmd == 'p':  print(f"  state: {mc.get_state()}")
            elif cmd == 'h':  print(HELP_TEXT)
            elif cmd in ('q', 'quit', 'exit'): break
            elif cmd in ('0','1','2','3','4','5','6','7'):
                mc.set_speed(int(cmd))
            elif cmd.startswith('m '):
                arg = cmd[2:].strip()
                if arg in ('pwm', 'full'):
                    mc.set_mode(MODE_PWM if arg == 'pwm' else MODE_FULL)
                else:
                    print(f"  unknown mode '{arg}' (use 'pwm' or 'full')")
            else:
                print(f"  unknown command: '{cmd}'  (type 'h' for help)")

    except KeyboardInterrupt:
        print()

    finally:
        mc.cleanup()
        print("Goodbye.")


if __name__ == '__main__':
    main()
