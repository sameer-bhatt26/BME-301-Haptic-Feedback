// ============================================================
//  MotorControl.cpp
//  Band energy → GPIO pin toggling for ERM haptic motors.
//
//  Uses libgpiod v2 (the modern, kernel-supported GPIO library).
//  pigpio and lgpio are NOT used — both are unmaintained and
//  unavailable on Debian Trixie / recent Raspberry Pi OS.
//
//  Hardware: 15 ERM motors driven via transistors on BCM pins:
//    2, 3, 14, 15, 17, 18, 27, 22, 23, 24, 10, 9, 25, 11, 8
//
//  Install:
//    sudo apt install libgpiod-dev libgpiod3
//
//  Compile (on the Pi, from the src/ directory):
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -lgpiod -lportaudio -lm
//
//  Run (no sudo needed — libgpiod uses the kernel character device):
//    ./HapticSleeve
// ============================================================

#pragma once

#include "AudioEngine.cpp"
#include <gpiod.h>
#include <array>
#include <algorithm>
#include <iostream>
#include <cstring>

// ─────────────────────────────────────────────────────────────
//  Hardware config
// ─────────────────────────────────────────────────────────────

// GPIO chip device — on all Pi models this is gpiochip0.
static constexpr const char* GPIO_CHIP = "/dev/gpiochip0";

// BCM pin numbers — matches the Python MOTOR_PINS_BCM list exactly.
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

// Energy threshold — AudioEngine envelopes are normalised to [0, 1].
// Start at 0.05 and tune up if motors fire too easily,
// or down if they don't fire enough.
static constexpr float ENVELOPE_THRESHOLD = 0.05f;

// Max simultaneous motors on — prevents current spikes on the Pi's 5V rail.
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
//
//  libgpiod v2 uses a "request" model:
//    1. Open the chip        → gpiod_chip_open()
//    2. Build a line config  → gpiod_line_config + gpiod_line_settings
//    3. Request all lines    → gpiod_chip_request_lines()
//    4. Write values         → gpiod_line_request_set_values_subset()
//    5. Release on shutdown  → gpiod_line_request_release()
//
//  All 15 motor lines are requested in one batch, which is more
//  efficient than requesting them individually.
// ─────────────────────────────────────────────────────────────

class MotorController {
public:

    // ── Init / shutdown ──────────────────────────────────────

    bool init() {
        // Open GPIO chip
        chip_ = gpiod_chip_open(GPIO_CHIP);
        if (!chip_) {
            std::cerr << "[MotorControl] Failed to open " << GPIO_CHIP << "\n"
                      << "  Check it exists:  ls -la /dev/gpiochip*\n"
                      << "  Add gpio group:   sudo usermod -aG gpio $USER\n"
                      << "  Then log out and back in.\n";
            return false;
        }

        // Line settings: output, initially LOW (motor off)
        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) {
            std::cerr << "[MotorControl] Failed to allocate line settings.\n";
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            return false;
        }
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        // Line config: apply the same settings to all motor pins
        gpiod_line_config* lineCfg = gpiod_line_config_new();
        if (!lineCfg) {
            std::cerr << "[MotorControl] Failed to allocate line config.\n";
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            return false;
        }
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const unsigned int offset = static_cast<unsigned int>(MOTOR_PINS[m]);
            gpiod_line_config_add_line_settings(lineCfg, &offset, 1, settings);
        }

        // Request config — consumer name shows in gpioinfo for debugging
        gpiod_request_config* reqCfg = gpiod_request_config_new();
        if (reqCfg)
            gpiod_request_config_set_consumer(reqCfg, "haptic-sleeve");

        // Request all lines in one kernel call
        request_ = gpiod_chip_request_lines(chip_, reqCfg, lineCfg);

        // Free config objects — not needed after request is made
        gpiod_line_settings_free(settings);
        gpiod_line_config_free(lineCfg);
        if (reqCfg) gpiod_request_config_free(reqCfg);

        if (!request_) {
            std::cerr << "[MotorControl] Failed to request GPIO lines.\n"
                      << "  Are any pins already claimed? Check with: gpioinfo\n";
            gpiod_chip_close(chip_);
            chip_ = nullptr;
            return false;
        }

        initialised_ = true;
        std::cout << "[MotorControl] " << MOTOR_COUNT
                  << " motors initialised on " << GPIO_CHIP << ".\n";
        return true;
    }

    void shutdown() {
        if (!initialised_) return;
        allOff();
        gpiod_line_request_release(request_);
        gpiod_chip_close(chip_);
        request_     = nullptr;
        chip_        = nullptr;
        initialised_ = false;
        std::cout << "[MotorControl] GPIO released.\n";
    }

    ~MotorController() { shutdown(); }

    // ── Main update — call once per audio buffer ─────────────

    /**
     * @brief  Translates AudioEngine FilterOutput into GPIO state.
     *
     * Logic mirrors the original Python script:
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
        struct Candidate { int motorIdx; float energy; };
        Candidate candidates[MOTOR_COUNT];
        int nCandidates = 0;

        for (int b = 0; b < BAND_COUNT; ++b) {
            const float env = out.bands[b].envelope;
            if (env > ENVELOPE_THRESHOLD) {
                const MotorRange& r = BAND_MOTOR_RANGES[b];
                for (int m = r.first; m <= r.last; ++m)
                    candidates[nCandidates++] = { m, env };
            }
        }

        // Sort by energy descending — strongest band fires first
        std::sort(candidates, candidates + nCandidates,
                  [](const Candidate& a, const Candidate& b) {
                      return a.energy > b.energy;
                  });

        // Determine which motors should be ON
        bool motorOn[MOTOR_COUNT] = {};
        const int limit = std::min(nCandidates, MAX_MOTORS_ON);
        for (int i = 0; i < limit; ++i)
            motorOn[candidates[i].motorIdx] = true;

        // Only write pins whose state has changed — batched into one kernel call
        unsigned int    changedOffsets[MOTOR_COUNT];
        gpiod_line_value changedValues[MOTOR_COUNT];
        int nChanged = 0;

        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const int desired = motorOn[m] ? 1 : 0;
            if (desired != pinState_[m]) {
                changedOffsets[nChanged] = static_cast<unsigned int>(MOTOR_PINS[m]);
                changedValues[nChanged]  = motorOn[m]
                                               ? GPIOD_LINE_VALUE_ACTIVE
                                               : GPIOD_LINE_VALUE_INACTIVE;
                pinState_[m] = desired;
                ++nChanged;
            }
        }

        if (nChanged > 0) {
            gpiod_line_request_set_values_subset(
                request_, nChanged, changedOffsets, changedValues);
        }
    }

    // ── Utility ──────────────────────────────────────────────

    void allOff() {
        if (!initialised_) return;
        unsigned int     offsets[MOTOR_COUNT];
        gpiod_line_value values[MOTOR_COUNT];
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            offsets[m]   = static_cast<unsigned int>(MOTOR_PINS[m]);
            values[m]    = GPIOD_LINE_VALUE_INACTIVE;
            pinState_[m] = 0;
        }
        gpiod_line_request_set_values_subset(
            request_, MOTOR_COUNT, offsets, values);
    }

    // Print current motor states to stdout for debugging
    void printState() const {
        std::cout << "[Motors] ";
        for (int m = 0; m < MOTOR_COUNT; ++m)
            std::cout << (pinState_[m] ? "█" : "░");
        std::cout << "  (█=ON  ░=OFF)\n";
    }

private:
    bool                 initialised_ = false;
    gpiod_chip*          chip_        = nullptr;
    gpiod_line_request*  request_     = nullptr;
    int                  pinState_[MOTOR_COUNT] = {};
};