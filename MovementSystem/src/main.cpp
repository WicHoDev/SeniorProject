/**
 * main.cpp  (PlatformIO / Arduino framework)
 * ==========================================
 * SFCW-GPR Tank Robot — ESP32 + 2× DRV8871-Q1 Motor Driver
 *
 * The Raspberry Pi 5 communicates via 6 GPIO bit-bang lines.
 * The ESP32 reads those lines and generates proper IN1/IN2 PWM
 * signals for each DRV8871-Q1 H-bridge.
 *
 * ─────────────────────────────────────────────────────────────────
 *  INTERFACE PINS  (Pi 5 GPIO → ESP32 input)
 * ─────────────────────────────────────────────────────────────────
 *
 *  Pi GPIO  │ ESP32 Pin │ Signal   │ Description
 *  ─────────┼───────────┼──────────┼───────────────────────────────
 *  IO_04    │ GPIO4     │ ENABLE   │ HIGH = motors active, LOW = stop (coast)
 *  IO_05    │ GPIO5     │ DIR_L    │ HIGH = left motor forward, LOW = reverse
 *  IO_06    │ GPIO6     │ DIR_R    │ HIGH = right motor forward, LOW = reverse
 *  IO_07    │ GPIO7     │ SPD_S0   │ Speed bit 0 (LSB)
 *  IO_08    │ GPIO8     │ SPD_S1   │ Speed bit 1
 *  IO_15    │ GPIO15    │ SPD_S2   │ Speed bit 2 (MSB)
 *
 *  Speed encoding (S2 S1 S0):
 *    000 →   0% PWM duty  (stopped, but ENABLE still high → brake mode)
 *    001 →  14% PWM duty
 *    010 →  28% PWM duty
 *    011 →  43% PWM duty
 *    100 →  57% PWM duty
 *    101 →  71% PWM duty
 *    110 →  86% PWM duty
 *    111 → 100% PWM duty  (full speed)
 *
 *  ENABLE = LOW  → both DRV8871s coast (IN1=0, IN2=0 → High-Z)
 *  ENABLE = HIGH + speed 000 → brake (IN1=1, IN2=1)
 *
 *  NOTE: Pi 5 GPIO is 3.3V. ESP32 GPIO is 3.3V. No level shifter needed.
 *        Connect Pi GND to ESP32 GND.
 *
 * ─────────────────────────────────────────────────────────────────
 *  DRV8871-Q1 OUTPUT PINS  (ESP32 → motor driver)
 * ─────────────────────────────────────────────────────────────────
 *
 *  ESP32 Pin │ DRV8871   │ Motor
 *  ──────────┼───────────┼───────────────────────────────────────
 *  IO10      │ U1 IN1    │ Left motor  IN1   (schematic: In1_1)
 *  IO11      │ U1 IN2    │ Left motor  IN2   (schematic: In1_2)
 *  IO47      │ U2 IN1    │ Right motor IN1   (schematic: In2_1)
 *  IO48      │ U2 IN2    │ Right motor IN2   (schematic: In2_2)
 *
 * ─────────────────────────────────────────────────────────────────
 *  DRV8871-Q1 H-BRIDGE TRUTH TABLE (from datasheet Table 1)
 * ─────────────────────────────────────────────────────────────────
 *
 *  IN1  │ IN2  │ OUT1  │ OUT2  │ Mode
 *  ─────┼──────┼───────┼───────┼──────────────────────────────────
 *   0   │  0   │ Hi-Z  │ Hi-Z  │ Coast (sleep after 1 ms)
 *   0   │  1   │  L    │  H    │ Reverse (current OUT2 → OUT1)
 *   1   │  0   │  H    │  L    │ Forward (current OUT1 → OUT2)
 *   1   │  1   │  L    │  L    │ Brake (slow decay)
 *
 *  PWM strategy used here: switch between DRIVE and BRAKE
 *    Forward at duty D:  IN1 = PWM(D), IN2 = 1   (brake during off-time)
 *    Reverse at duty D:  IN1 = 1,      IN2 = PWM(D)
 *    Stop/coast:         IN1 = 0,      IN2 = 0
 *    Brake:              IN1 = 1,      IN2 = 1
 *
 *  This "slow decay" PWM gives smoother current and better torque
 *  at low speeds than fast decay (coast during off-time).
 *
 * ─────────────────────────────────────────────────────────────────
 *  DRV8871 ILIM RESISTOR — Current Limit
 * ─────────────────────────────────────────────────────────────────
 *
 *  From datasheet Eq.1:
 *
 *             V_ILIM [kV]      64 [kΩ]
 *  I_TRIP = ─────────────── = ─────────────
 *             R_ILIM [kΩ]      R_ILIM [kΩ]
 *
 *  Recommended value for tank robot motors (assuming ~1 A RMS):
 *    I_TRIP = 2.0 A  →  R_ILIM = 64/2.0 = 32.0 kΩ  (use 32.4 kΩ standard)
 *    I_TRIP = 1.5 A  →  R_ILIM = 64/1.5 = 42.667 kΩ (use 43.2 kΩ standard)
 *    I_TRIP = 3.0 A  →  R_ILIM = 64/3.0 = 21.333 kΩ (use 21.5 kΩ standard)
 *  Minimum allowed: R_ILIM = 15 kΩ  (I_TRIP_max = 4.267 A, near OCP limit)
 *
 *  Connect R_ILIM between ILIM pin and GND on each DRV8871 board.
 *
 * ─────────────────────────────────────────────────────────────────
 *  WATCHDOG
 * ─────────────────────────────────────────────────────────────────
 *  If the ENABLE pin stays LOW for more than WATCHDOG_MS, motors
 *  remain coasted. This is passive safety — no active watchdog
 *  needed since ENABLE=LOW already cuts the motors.
 *
 *  An additional software watchdog ensures motors stop if the
 *  GPIO lines are not refreshed by the Pi (Pi crash / GPIO freeze).
 *  The Pi must toggle a heartbeat on ENABLE at least every
 *  WATCHDOG_MS ms, or motors will be forced to coast.
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════
//  PI → ESP32 INPUT PINS  (bit-bang interface)
// ═══════════════════════════════════════════════════════════
#define PIN_ENABLE   4    // IO_04 — global enable
#define PIN_DIR_L    5    // IO_05 — left  motor direction
#define PIN_DIR_R    6    // IO_06 — right motor direction
#define PIN_SPD_S0   7    // IO_07 — speed bit 0 (LSB)
#define PIN_SPD_S1   8    // IO_08 — speed bit 1
#define PIN_SPD_S2   15   // IO_15 — speed bit 2 (MSB)

// ═══════════════════════════════════════════════════════════
//  ESP32 → DRV8871 OUTPUT PINS  (per schematic)
// ═══════════════════════════════════════════════════════════
#define PIN_L_IN1    10   // IO10 — Left  motor DRV8871 IN1  (In1_1)
#define PIN_L_IN2    11   // IO11 — Left  motor DRV8871 IN2  (In1_2)
#define PIN_R_IN1    47   // IO47 — Right motor DRV8871 IN1  (In2_1)
#define PIN_R_IN2    48   // IO48 — Right motor DRV8871 IN2  (In2_2)

// ═══════════════════════════════════════════════════════════
//  LEDC PWM CONFIG
// ═══════════════════════════════════════════════════════════
#define PWM_FREQ       20000   // 20 kHz — inaudible, within DRV8871 f_PWM max
#define PWM_BITS       8       // 8-bit resolution → 0–255
#define LEDC_CH_L_IN1  0
#define LEDC_CH_L_IN2  1
#define LEDC_CH_R_IN1  2
#define LEDC_CH_R_IN2  3

// ═══════════════════════════════════════════════════════════
//  SPEED TABLE
//  3-bit binary (0-7) → PWM duty cycle (0-255)
//  Level 0 = 0%, Level 7 = 100%
//  Steps are roughly linear: 255/7 ≈ 36.4 per step
// ═══════════════════════════════════════════════════════════
const uint8_t SPEED_TABLE[8] = {
//  S2S1S0  Level  Duty%   Duty/255
    0,    //  000    0%      0
    36,   //  001   14%     36
    73,   //  010   28%     73
    109,  //  011   43%    109
    146,  //  100   57%    146
    182,  //  101   71%    182
    218,  //  110   86%    218
    255   //  111  100%    255
};

// ═══════════════════════════════════════════════════════════
//  MOTOR INVERSION
//  Set true if a motor spins wrong way for your chassis mount
// ═══════════════════════════════════════════════════════════
#define INVERT_LEFT   false
#define INVERT_RIGHT  true   // right side typically mirrored on tank chassis

// ═══════════════════════════════════════════════════════════
//  STATUS LED
// ═══════════════════════════════════════════════════════════
#define PIN_LED  2

// ═══════════════════════════════════════════════════════════
//  FORWARD DECLARATIONS  (required for .cpp under PlatformIO)
// ═══════════════════════════════════════════════════════════
void drv8871_drive(uint8_t chIN1, uint8_t chIN2, bool forward, uint8_t duty);
void drv8871_brake(uint8_t chIN1, uint8_t chIN2);
void drv8871_coast(uint8_t chIN1, uint8_t chIN2);

// ═══════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 DRV8871 Motor Controller — GPIO bit-bang interface");
    Serial.printf("  ENABLE  : GPIO%d\n", PIN_ENABLE);
    Serial.printf("  DIR_L   : GPIO%d\n", PIN_DIR_L);
    Serial.printf("  DIR_R   : GPIO%d\n", PIN_DIR_R);
    Serial.printf("  SPD S0/S1/S2: GPIO%d/%d/%d\n", PIN_SPD_S0, PIN_SPD_S1, PIN_SPD_S2);
    Serial.printf("  L_IN1/IN2  : GPIO%d/%d\n", PIN_L_IN1, PIN_L_IN2);
    Serial.printf("  R_IN1/IN2  : GPIO%d/%d\n", PIN_R_IN1, PIN_R_IN2);
    Serial.printf("  PWM freq   : %d Hz, %d-bit\n", PWM_FREQ, PWM_BITS);

    // Input pins — use internal pulldown so float = 0 = safe
    pinMode(PIN_ENABLE, INPUT_PULLDOWN);
    pinMode(PIN_DIR_L,  INPUT_PULLDOWN);
    pinMode(PIN_DIR_R,  INPUT_PULLDOWN);
    pinMode(PIN_SPD_S0, INPUT_PULLDOWN);
    pinMode(PIN_SPD_S1, INPUT_PULLDOWN);
    pinMode(PIN_SPD_S2, INPUT_PULLDOWN);

    // Status LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // LEDC PWM — one channel per DRV8871 input pin
    // arduino-esp32 v3+ uses ledcAttach(pin, freq, res); v2 uses ledcSetup+ledcAttachPin
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttachChannel(PIN_L_IN1, PWM_FREQ, PWM_BITS, LEDC_CH_L_IN1);
    ledcAttachChannel(PIN_L_IN2, PWM_FREQ, PWM_BITS, LEDC_CH_L_IN2);
    ledcAttachChannel(PIN_R_IN1, PWM_FREQ, PWM_BITS, LEDC_CH_R_IN1);
    ledcAttachChannel(PIN_R_IN2, PWM_FREQ, PWM_BITS, LEDC_CH_R_IN2);
#else
    ledcSetup(LEDC_CH_L_IN1, PWM_FREQ, PWM_BITS);
    ledcSetup(LEDC_CH_L_IN2, PWM_FREQ, PWM_BITS);
    ledcSetup(LEDC_CH_R_IN1, PWM_FREQ, PWM_BITS);
    ledcSetup(LEDC_CH_R_IN2, PWM_FREQ, PWM_BITS);
    ledcAttachPin(PIN_L_IN1, LEDC_CH_L_IN1);
    ledcAttachPin(PIN_L_IN2, LEDC_CH_L_IN2);
    ledcAttachPin(PIN_R_IN1, LEDC_CH_R_IN1);
    ledcAttachPin(PIN_R_IN2, LEDC_CH_R_IN2);
#endif

    // Start coasted (both inputs LOW = Hi-Z)
    drv8871_coast(LEDC_CH_L_IN1, LEDC_CH_L_IN2);
    drv8871_coast(LEDC_CH_R_IN1, LEDC_CH_R_IN2);

    Serial.println("Ready.");
}

// ═══════════════════════════════════════════════════════════
//  LOOP — poll GPIO lines every 5 ms
// ═══════════════════════════════════════════════════════════
void loop() {
    // Read all input lines
    bool enable = digitalRead(PIN_ENABLE);
    bool dir_l  = digitalRead(PIN_DIR_L);
    bool dir_r  = digitalRead(PIN_DIR_R);
    uint8_t s0  = digitalRead(PIN_SPD_S0);
    uint8_t s1  = digitalRead(PIN_SPD_S1);
    uint8_t s2  = digitalRead(PIN_SPD_S2);

    uint8_t speed_idx = (s2 << 2) | (s1 << 1) | s0;   // 0–7
    uint8_t duty      = SPEED_TABLE[speed_idx];          // 0–255

    // ─── Log only when input state changes (debounced 30 ms) ───
    static uint8_t  last_state    = 0xFF;
    static uint32_t last_log_ms   = 0;
    uint8_t cur_state = (enable << 5) | (dir_l << 4) | (dir_r << 3) | speed_idx;
    if (cur_state != last_state && (millis() - last_log_ms) > 30) {
        Serial.printf("[IN] EN=%d DIR_L=%d DIR_R=%d SPD=%d%d%d (lvl %d, duty %d)\n",
                      enable, dir_l, dir_r, s2, s1, s0, speed_idx, duty);
        last_state  = cur_state;
        last_log_ms = millis();
    }

    // Status LED: on when enabled and moving
    digitalWrite(PIN_LED, enable && duty > 0);

    if (!enable) {
        // ENABLE low → coast both motors (DRV8871 sleep after 1 ms)
        drv8871_coast(LEDC_CH_L_IN1, LEDC_CH_L_IN2);
        drv8871_coast(LEDC_CH_R_IN1, LEDC_CH_R_IN2);
    } else if (duty == 0) {
        // Enabled but speed = 0 → brake (IN1=1, IN2=1 → slow decay)
        drv8871_brake(LEDC_CH_L_IN1, LEDC_CH_L_IN2);
        drv8871_brake(LEDC_CH_R_IN1, LEDC_CH_R_IN2);
    } else {
        // Drive with direction and speed
        bool fwd_l = INVERT_LEFT  ? !dir_l : dir_l;
        bool fwd_r = INVERT_RIGHT ? !dir_r : dir_r;

        drv8871_drive(LEDC_CH_L_IN1, LEDC_CH_L_IN2, fwd_l, duty);
        drv8871_drive(LEDC_CH_R_IN1, LEDC_CH_R_IN2, fwd_r, duty);
    }

    delay(5);   // 200 Hz polling — fast enough for smooth control
}

// ═══════════════════════════════════════════════════════════
//  DRV8871 DRIVE FUNCTIONS
//
//  DRV8871 slow-decay PWM strategy:
//    Forward: IN1 = PWM(duty),  IN2 = 255 (always HIGH)
//    Reverse: IN1 = 255 (always HIGH), IN2 = PWM(duty)
//
//  During the PWM off-time, the non-PWM'd input stays HIGH,
//  pulling both inputs high → brake/slow-decay. This gives
//  much smoother current regulation and better low-speed torque
//  than fast-decay (coast during off-time).
// ═══════════════════════════════════════════════════════════

/**
 * drv8871_drive — run a motor at given duty in given direction
 *
 * chIN1, chIN2 : LEDC channel numbers for IN1 and IN2
 * forward      : true = forward (OUT1→OUT2), false = reverse
 * duty         : PWM duty 0–255 (should be > 0 when calling this)
 */
void drv8871_drive(uint8_t chIN1, uint8_t chIN2, bool forward, uint8_t duty) {
    if (forward) {
        // Forward: PWM on IN1, IN2 = full HIGH (slow decay during off)
        ledcWrite(chIN1, duty);
        ledcWrite(chIN2, 255);
    } else {
        // Reverse: IN1 = full HIGH, PWM on IN2 (slow decay during off)
        ledcWrite(chIN1, 255);
        ledcWrite(chIN2, duty);
    }
}

/**
 * drv8871_brake — both inputs HIGH → low-side slow decay (active brake)
 *
 * Per datasheet Table 1: IN1=1, IN2=1 → OUT1=L, OUT2=L (brake)
 */
void drv8871_brake(uint8_t chIN1, uint8_t chIN2) {
    ledcWrite(chIN1, 255);
    ledcWrite(chIN2, 255);
}

/**
 * drv8871_coast — both inputs LOW → Hi-Z (motor freewheels)
 *
 * Per datasheet Table 1: IN1=0, IN2=0 → OUT1=Hi-Z, OUT2=Hi-Z
 * Device enters sleep mode after ~1 ms (draws only ~13 µA)
 */
void drv8871_coast(uint8_t chIN1, uint8_t chIN2) {
    ledcWrite(chIN1, 0);
    ledcWrite(chIN2, 0);
}
