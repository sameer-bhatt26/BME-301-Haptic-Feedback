// ============================================================
//  MotorControl.cpp
//  Band energy → GPIO pin toggling for ERM haptic motors.
//
//  Mirrors the Python gpiozero logic exactly:
//    • Reads band envelopes from AudioEngine's FilterOutput
//    • Compares each band's envelope against THRESHOLD
//    • Drives the corresponding GPIO pins HIGH or LOW
//
//  Hardware: 15 ERM motors driven via transistors on BCM pins:
//    2, 3, 14, 15, 17, 18, 27, 22, 23, 24, 10, 9, 25, 11, 8
//
//  Requires pigpio:
//    sudo apt install pigpio
//    sudo pigpiod          ← must be running before launch
//
//  Compile (on the Pi):
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -lpigpio -lportaudio -lrt -lm
//
//  Run:
//    sudo ./HapticSleeve
// ============================================================

#pragma once

#include "AudioEngine.cpp"
#include <pigpio.h>
#include <array>
#include <algorithm>
#include <iostream>

// ─────────────────────────────────────────────────────────────
//  Hardware config
// ─────────────────────────────────────────────────────────────

// BCM pin numbers — matches the Python MOTOR_PINS_BCM list exactly.
// Index in this array = motor index used throughout this file.
static constexpr std::array<int, 15> MOTOR_PINS = {
     2,   // Motor  0  (Physical Pin  3)
     3,   // Motor  1  (Physical Pin  5)
    14,   // Motor  2  (Physical Pin  8)
    15,   // Motor  3  (Physical Pin 10)
    17,   // Motor  4  (Physical Pin 11)
    18,   // Motor  5  (Physical Pin 12)
    27,   // Motor  6  (Physical Pin 13)
    22,   // Motor  7  (Physical Pin 15)
    23,   // Motor  8  (Physical Pin 16)
    24,   // Motor  9  (Physical Pin 18)
    10,   // Motor 10  (Physical Pin 19)
     9,   // Motor 11  (Physical Pin 21)
    25,   // Motor 12  (Physical Pin 22)
    11,   // Motor 13  (Physical Pin 23)
     8,   // Motor 14  (Physical Pin 24)
};

static constexpr int MOTOR_COUNT = static_cast<int>(MOTOR_PINS.size());  // 15

// Energy threshold — equivalent to Python's THRESHOLD = 2e5.
// AudioEngine envelopes are normalised to [0, 1], so this is scaled
// accordingly. Start at 0.05 and tune up if motors fire too easily,
// or down if they don't fire enough.
static constexpr float ENVELOPE_THRESHOLD = 0.05f;

// Max simultaneous motors — mirrors Python's MAX_MOTORS_ON = 8.
// Prevents current spikes on the Pi's 5V rail when many motors fire at once.
static constexpr int MAX_MOTORS_ON = 8;


// ─────────────────────────────────────────────────────────────
//  Band → Motor mapping
//
//  6 frequency bands spread across 15 motors.
//  Lower bands get more motors — ERMs are most effective at low freq.
//
//  BAND_MOTOR_RANGES[b] = { first_motor_index, last_motor_index }
//  (inclusive on both ends)
// ─────────────────────────────────────────────────────────────

struct MotorRange { int first; int last; };

static constexpr std::array<MotorRange, BAND_COUNT> BAND_MOTOR_RANGES = {{
    { 0,  2  },   // BAND_SUB_BASS  →  motors 0–2   (3 motors, most impactful)
    { 3,  5  },   // BAND_BASS      →  motors 3–5   (3 motors)
    { 6,  8  },   // BAND_LOW_MID   →  motors 6–8   (3 motors)
    { 9,  10 },   // BAND_MID       →  motors 9–10  (2 motors)
    { 11, 12 },   // BAND_HIGH_MID  →  motors 11–12 (2 motors)
    { 13, 14 },   // BAND_PRESENCE  →  motors 13–14 (2 motors, least tactile)
}};


// ─────────────────────────────────────────────────────────────
//  MotorController
// ─────────────────────────────────────────────────────────────

class MotorController {
public:

    // ── Init / shutdown ──────────────────────────────────────

    bool init() {
        if (gpioInitialise() < 0) {
            std::cerr << "[MotorControl] pigpio init failed. "
                         "Is pigpiod running? Try: sudo pigpiod\n";
            return false;
        }

        for (int pin : MOTOR_PINS) {
            gpioSetMode(pin, PI_OUTPUT);
            gpioWrite(pin, 0);   // start LOW (motor off)
        }

        initialised_ = true;
        std::cout << "[MotorControl] " << MOTOR_COUNT
                  << " motors initialised on BCM pins.\n";
        return true;
    }

    void shutdown() {
        if (!initialised_) return;
        allOff();
        gpioTerminate();
        initialised_ = false;
        std::cout << "[MotorControl] GPIO released.\n";
    }

    ~MotorController() { shutdown(); }

    // ── Main update — call this every audio buffer ───────────

    /**
     * @brief  Translates AudioEngine output into GPIO state.
     *
     * Logic mirrors the Python script:
     *   1. For each band, check if envelope > threshold
     *   2. Collect all candidate motors whose band is active
     *   3. Sort by band energy (strongest first)
     *   4. Turn on top MAX_MOTORS_ON, turn off the rest
     *
     * @param out   FilterOutput from AudioEngine::process()
     */
    void update(const FilterOutput& out) {
        if (!initialised_) return;

        // Build candidate list: (motor_index, band_envelope)
        // A motor is a candidate if its band's envelope exceeds the threshold.
        struct Candidate { int motorIdx; float energy; };
        Candidate candidates[MOTOR_COUNT];
        int nCandidates = 0;

        for (int b = 0; b < BAND_COUNT; ++b) {
            const float env = out.bands[b].envelope;
            if (env > ENVELOPE_THRESHOLD) {
                const MotorRange& r = BAND_MOTOR_RANGES[b];
                for (int m = r.first; m <= r.last; ++m) {
                    candidates[nCandidates++] = { m, env };
                }
            }
        }

        // Sort candidates by energy descending (strongest band first)
        std::sort(candidates, candidates + nCandidates,
                  [](const Candidate& a, const Candidate& b) {
                      return a.energy > b.energy;
                  });

        // Build set of motors to turn ON (top MAX_MOTORS_ON)
        bool motorOn[MOTOR_COUNT] = {};
        const int limit = std::min(nCandidates, MAX_MOTORS_ON);
        for (int i = 0; i < limit; ++i)
            motorOn[candidates[i].motorIdx] = true;

        // Apply to GPIO — only write if state changed (reduces bus traffic)
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const int desired = motorOn[m] ? 1 : 0;
            if (desired != pinState_[m]) {
                gpioWrite(MOTOR_PINS[m], desired);
                pinState_[m] = desired;
            }
        }
    }

    // ── Utility ──────────────────────────────────────────────

    void allOff() {
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            gpioWrite(MOTOR_PINS[m], 0);
            pinState_[m] = 0;
        }
    }

    // Print current motor states to stdout for debugging
    void printState() const {
        std::cout << "[Motors] ";
        for (int m = 0; m < MOTOR_COUNT; ++m)
            std::cout << (pinState_[m] ? "█" : "░");
        std::cout << "  (█=ON ░=OFF)\n";
    }

private:
    bool initialised_           = false;
    int  pinState_[MOTOR_COUNT] = {};   // last written state per motor
};