#pragma once

// ============================================================
//  MotorControl.cpp
//  Hardware-only GPIO driver for ERM haptic motors.
//
//  This file knows NOTHING about audio or frequency mapping.
//  It takes a MotorCommand (bool[12]) and writes GPIO pins.
//  All mapping logic lives in Mappers.cpp.
//
//  Uses libgpiod v2 — install with:
//    sudo apt install libgpiod-dev libgpiod3
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -lgpiod -lportaudio -lm
//
//  No sudo needed — libgpiod uses the kernel character device.
//  User must be in the gpio group:
//    sudo usermod -aG gpio $USER  (then log out and back in)
// ============================================================

#include "Mappers.cpp"
#include <gpiod.h>
#include <iostream>
#include <array>
#include <algorithm>

// ─────────────────────────────────────────────────────────────
//  GPIO chip
//  gpiochip0 is correct for all current Raspberry Pi models.
// ─────────────────────────────────────────────────────────────

static constexpr const char* GPIO_CHIP = "/dev/gpiochip0";


// ─────────────────────────────────────────────────────────────
//  Motor → BCM pin mapping
//
//  Index in this array = motor index (0–11).
//  Motor layout on the sleeve:
//
//      Motor 0   Motor 1   Motor 2   Motor 3   ← row 0 (low freq)
//      Motor 4   Motor 5   Motor 6   Motor 7   ← row 1 (mid freq)
//      Motor 8   Motor 9   Motor 10  Motor 11  ← row 2 (high freq)
//
//  Update BCM pin numbers here after any circuit revision.
//  Physical pin numbers are noted in comments for reference.
// ─────────────────────────────────────────────────────────────

static constexpr std::array<int, BAND_COUNT> MOTOR_PINS = {
     2,   // Motor  0  — row 0, col 0  (Physical Pin  3)
     3,   // Motor  1  — row 0, col 1  (Physical Pin  5)
    14,   // Motor  2  — row 0, col 2  (Physical Pin  8)
    15,   // Motor  3  — row 0, col 3  (Physical Pin 10)
    17,   // Motor  4  — row 1, col 0  (Physical Pin 11)
    18,   // Motor  5  — row 1, col 1  (Physical Pin 12)
    27,   // Motor  6  — row 1, col 2  (Physical Pin 13)
    22,   // Motor  7  — row 1, col 3  (Physical Pin 15)
    23,   // Motor  8  — row 2, col 0  (Physical Pin 16)
    24,   // Motor  9  — row 2, col 1  (Physical Pin 18)
    10,   // Motor 10  — row 2, col 2  (Physical Pin 19)
     9,   // Motor 11  — row 2, col 3  (Physical Pin 21)
};

static constexpr int MOTOR_COUNT = static_cast<int>(MOTOR_PINS.size());
static_assert(MOTOR_COUNT == BAND_COUNT,
    "MOTOR_PINS size must equal BAND_COUNT — one pin per band/motor.");


// ─────────────────────────────────────────────────────────────
//  MotorController
// ─────────────────────────────────────────────────────────────

class MotorController {
public:

    // ── Init / shutdown ──────────────────────────────────────

    bool init() {
        chip_ = gpiod_chip_open(GPIO_CHIP);
        if (!chip_) {
            std::cerr << "[MotorControl] Cannot open " << GPIO_CHIP << "\n"
                      << "  ls -la /dev/gpiochip*          ← check it exists\n"
                      << "  sudo usermod -aG gpio $USER    ← add permissions\n"
                      << "  (log out and back in after usermod)\n";
            return false;
        }

        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) {
            std::cerr << "[MotorControl] Failed to alloc line settings.\n";
            gpiod_chip_close(chip_); chip_ = nullptr;
            return false;
        }
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

        gpiod_line_config* lineCfg = gpiod_line_config_new();
        if (!lineCfg) {
            std::cerr << "[MotorControl] Failed to alloc line config.\n";
            gpiod_line_settings_free(settings);
            gpiod_chip_close(chip_); chip_ = nullptr;
            return false;
        }
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const unsigned int offset = static_cast<unsigned int>(MOTOR_PINS[m]);
            gpiod_line_config_add_line_settings(lineCfg, &offset, 1, settings);
        }

        gpiod_request_config* reqCfg = gpiod_request_config_new();
        if (reqCfg) gpiod_request_config_set_consumer(reqCfg, "haptic-sleeve");

        request_ = gpiod_chip_request_lines(chip_, reqCfg, lineCfg);

        gpiod_line_settings_free(settings);
        gpiod_line_config_free(lineCfg);
        if (reqCfg) gpiod_request_config_free(reqCfg);

        if (!request_) {
            std::cerr << "[MotorControl] Failed to request GPIO lines.\n"
                      << "  Are pins already claimed?  Run: gpioinfo\n";
            gpiod_chip_close(chip_); chip_ = nullptr;
            return false;
        }

        initialised_ = true;
        std::cout << "[MotorControl] " << MOTOR_COUNT
                  << " motors ready on " << GPIO_CHIP << ".\n";
        return true;
    }

    void shutdown() {
        if (!initialised_) return;
        allOff();
        gpiod_line_request_release(request_);
        gpiod_chip_close(chip_);
        request_ = nullptr; chip_ = nullptr;
        initialised_ = false;
        std::cout << "[MotorControl] GPIO released.\n";
    }

    ~MotorController() { shutdown(); }

    // ── Drive motors from a MotorCommand ────────────────────

    /**
     * @brief  Write GPIO pins based on a MotorCommand from any mapper.
     *         Only writes pins whose state has changed — reduces
     *         kernel call overhead and GPIO bus traffic.
     *
     * @param cmd   MotorCommand produced by IMapper::map()
     */
    void update(const MotorCommand& cmd) {
        if (!initialised_) return;

        unsigned int     changedOffsets[MOTOR_COUNT];
        gpiod_line_value changedValues [MOTOR_COUNT];
        int nChanged = 0;

        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const int desired = cmd.active[m] ? 1 : 0;
            if (desired != pinState_[m]) {
                changedOffsets[nChanged] = static_cast<unsigned int>(MOTOR_PINS[m]);
                changedValues [nChanged] = cmd.active[m]
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
        gpiod_line_value values [MOTOR_COUNT];
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            offsets[m]   = static_cast<unsigned int>(MOTOR_PINS[m]);
            values [m]   = GPIOD_LINE_VALUE_INACTIVE;
            pinState_[m] = 0;
        }
        gpiod_line_request_set_values_subset(
            request_, MOTOR_COUNT, offsets, values);
    }

    void printState() const {
        std::cout << "[Motors] ";
        for (int m = 0; m < MOTOR_COUNT; ++m)
            std::cout << (pinState_[m] ? "█" : "░");
        std::cout << "  (█=ON  ░=OFF)\n";
    }

private:
    bool                initialised_ = false;
    gpiod_chip*         chip_        = nullptr;
    gpiod_line_request* request_     = nullptr;
    int                 pinState_[MOTOR_COUNT] = {};
};