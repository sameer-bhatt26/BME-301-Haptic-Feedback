#pragma once

// ============================================================
//  MotorControl.cpp
//  PCA9685 PWM driver for ERM haptic motors via I2C.
//
//  Hardware path:
//    Pi Zero 2 W  ->  PCA9685 (I2C)  ->  ULN2803A  ->  ERM motors
//
//  The ULN2803A is an active-high inverting Darlington driver:
//    PCA9685 output HIGH -> ULN2803A pulls motor to GND -> motor ON
//    PCA9685 output LOW  -> motor floating              -> motor OFF
//  So high PWM duty cycle = motor spinning. No inversion needed.
//
//  Install:
//    sudo apt install libi2c-dev
//    sudo raspi-config -> Interface Options -> I2C -> Enable
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -li2c -lportaudio -lm
//
//  First-run checks:
//    i2cdetect -y 1          <- should show 0x40 (or your address)
//    ls /dev/i2c*            <- should show /dev/i2c-1
// ============================================================

#include "Mappers.cpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <cstdint>
#include <cmath>
#include <iostream>
#include <array>
#include <algorithm>
#include <chrono>
#include <thread>


// ─────────────────────────────────────────────────────────────
//  I2C / PCA9685 config
//  These are the only values you need to change if your board
//  uses a different I2C bus or I2C address.
// ─────────────────────────────────────────────────────────────

// I2C bus — /dev/i2c-1 on all current Raspberry Pi models
static constexpr const char* I2C_BUS     = "/dev/i2c-1";

// PCA9685 7-bit I2C address.  Default is 0x40.
// Run "i2cdetect -y 1" to confirm. Adjust if A0-A5 jumpers are set.
static constexpr uint8_t PCA9685_ADDR    = 0x40;

// PWM frequency in Hz. 1000 Hz works well for ERMs.
// Lower = more torque at low duty, higher = quieter.
static constexpr float   PWM_FREQ_HZ     = 1000.0f;

// PCA9685 internal oscillator (datasheet: 25 MHz)
static constexpr float   PCA9685_OSC_HZ  = 25000000.0f;

// 12-bit resolution: 0 = off, 4095 = full on
static constexpr int     PWM_MAX         = 4095;


// ─────────────────────────────────────────────────────────────
//  Motor -> PCA9685 channel mapping
//
//  MOTOR_CHANNELS[motor_index] = PCA9685 output channel (0-15)
//
//  Sleeve layout:
//    Motor 0   Motor 1   Motor 2   Motor 3   <- row 0 (low freq)
//    Motor 4   Motor 5   Motor 6   Motor 7   <- row 1 (mid freq)
//    Motor 8   Motor 9   Motor 10  Motor 11  <- row 2 (high freq)
//
//  PCA9685 channels used: 0-5 and 8-13.
//  Channels 6 and 7 are intentionally skipped (PCB routing decision).
//
//  If the physical motor order on the sleeve doesn't match the
//  frequency order you want, swap values in this array only —
//  no other file needs to change.
// ─────────────────────────────────────────────────────────────

static constexpr std::array<int, BAND_COUNT> MOTOR_CHANNELS = {
     0,   // Motor  0  — row 0, col 0  — PCA9685 ch 0
     1,   // Motor  1  — row 0, col 1  — PCA9685 ch 1
     2,   // Motor  2  — row 0, col 2  — PCA9685 ch 2
     3,   // Motor  3  — row 0, col 3  — PCA9685 ch 3
     4,   // Motor  4  — row 1, col 0  — PCA9685 ch 4
     5,   // Motor  5  — row 1, col 1  — PCA9685 ch 5
     8,   // Motor  6  — row 1, col 2  — PCA9685 ch 8  (ch 6 & 7 skipped)
     9,   // Motor  7  — row 1, col 3  — PCA9685 ch 9
    10,   // Motor  8  — row 2, col 0  — PCA9685 ch 10
    11,   // Motor  9  — row 2, col 1  — PCA9685 ch 11
    12,   // Motor 10  — row 2, col 2  — PCA9685 ch 12
    13,   // Motor 11  — row 2, col 3  — PCA9685 ch 13
};

static constexpr int MOTOR_COUNT = static_cast<int>(MOTOR_CHANNELS.size());
static_assert(MOTOR_COUNT == BAND_COUNT,
    "MOTOR_CHANNELS must have one entry per band/motor.");


// ─────────────────────────────────────────────────────────────
//  ERM overdrive (kickstart) parameters
//
//  ERMs need a brief burst of high power to overcome inertia
//  before settling to the proportional target level.
//
//  OVERDRIVE_FRAMES  — frames at 100% PWM on motor-start.
//                      1 frame ~= 33 ms at 30 Hz loop rate.
//                      Increase to 2 if motors fail to start.
//
//  MIN_SPIN_DUTY     — minimum 12-bit duty while motor is active.
//                      Below this the ERM won't spin even after
//                      kickstart. Tune per your specific motor.
//                      512 ~= 12.5% duty cycle.
// ─────────────────────────────────────────────────────────────

static constexpr int OVERDRIVE_FRAMES = 1;     // ~33 ms kickstart
static constexpr int MIN_SPIN_DUTY    = 512;   // ~12.5% minimum


// ─────────────────────────────────────────────────────────────
//  PCA9685 register definitions  (from datasheet)
// ─────────────────────────────────────────────────────────────

static constexpr uint8_t REG_MODE1       = 0x00;
static constexpr uint8_t REG_PRESCALE    = 0xFE;
static constexpr uint8_t REG_LED0_ON_L   = 0x06;  // base for channel 0
static constexpr uint8_t BYTES_PER_CH    = 4;      // ON_L, ON_H, OFF_L, OFF_H
static constexpr uint8_t MODE1_SLEEP     = 0x10;
static constexpr uint8_t MODE1_ALLCALL   = 0x01;
static constexpr uint8_t MODE1_AI        = 0x20;   // auto-increment


// ─────────────────────────────────────────────────────────────
//  MotorController
// ─────────────────────────────────────────────────────────────

class MotorController {
public:

    bool init() {
        // Open I2C bus
        i2cFd_ = open(I2C_BUS, O_RDWR);
        if (i2cFd_ < 0) {
            std::cerr << "[MotorControl] Cannot open " << I2C_BUS << "\n"
                      << "  Enable I2C: sudo raspi-config -> Interfaces -> I2C\n"
                      << "  Check bus:  ls /dev/i2c*\n";
            return false;
        }

        // Address the PCA9685
        if (ioctl(i2cFd_, I2C_SLAVE, PCA9685_ADDR) < 0) {
            std::cerr << "[MotorControl] Cannot reach PCA9685 at 0x"
                      << std::hex << (int)PCA9685_ADDR << std::dec << "\n"
                      << "  Scan bus: i2cdetect -y 1\n";
            close(i2cFd_); i2cFd_ = -1;
            return false;
        }

        if (!initPCA9685()) {
            close(i2cFd_); i2cFd_ = -1;
            return false;
        }

        initialised_ = true;
        std::cout << "[MotorControl] PCA9685 ready at 0x"
                  << std::hex << (int)PCA9685_ADDR << std::dec
                  << "  PWM: " << PWM_FREQ_HZ << " Hz\n";
        return true;
    }

    void shutdown() {
        if (!initialised_) return;
        allOff();
        writeReg(REG_MODE1, MODE1_SLEEP);
        close(i2cFd_);
        i2cFd_ = -1; initialised_ = false;
        std::cout << "[MotorControl] Shutdown.\n";
    }

    ~MotorController() { shutdown(); }

    // ── Main update ──────────────────────────────────────────

    void update(const MotorCommand& cmd) {
        if (!initialised_) return;

        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const float intensity = cmd.intensity[m];
            const bool  wantOn    = (intensity > 0.001f);
            int targetDuty = 0;

            if (wantOn) {
                targetDuty = static_cast<int>(intensity * PWM_MAX);

                // Kickstart: blast full power on first frame
                if (prevDuty_[m] == 0)
                    overdriveCounter_[m] = OVERDRIVE_FRAMES;

                if (overdriveCounter_[m] > 0) {
                    targetDuty = PWM_MAX;
                    --overdriveCounter_[m];
                } else {
                    // Clamp to minimum spin threshold
                    targetDuty = std::max(targetDuty, MIN_SPIN_DUTY);
                }
            } else {
                overdriveCounter_[m] = 0;
            }

            // Only write if changed — saves I2C bandwidth
            if (targetDuty != prevDuty_[m]) {
                setChannelDuty(MOTOR_CHANNELS[m], targetDuty);
                prevDuty_[m] = targetDuty;
            }
        }
    }

    void allOff() {
        if (!initialised_) return;
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            setChannelDuty(MOTOR_CHANNELS[m], 0);
            prevDuty_[m] = 0; overdriveCounter_[m] = 0;
        }
    }

    // Intensity bar display for console
    void printState(const MotorCommand& cmd) const {
        std::cout << "[Motors] ";
        for (int m = 0; m < MOTOR_COUNT; ++m) {
            const float v = cmd.intensity[m];
            if      (v > 0.75f) std::cout << "\xe2\x96\x88"; // █
            else if (v > 0.50f) std::cout << "\xe2\x96\x93"; // ▓
            else if (v > 0.25f) std::cout << "\xe2\x96\x92"; // ▒
            else if (v > 0.01f) std::cout << "\xe2\x96\x91"; // ░
            else                std::cout << " ";
        }
        std::cout << "  (full->low intensity: █▓▒░)\n";
    }

private:

    bool initPCA9685() {
        // Sleep mode required before setting prescale
        if (!writeReg(REG_MODE1, MODE1_SLEEP)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        // prescale = round(osc / (4096 * freq)) - 1
        const uint8_t prescale = static_cast<uint8_t>(
            std::round(PCA9685_OSC_HZ / (4096.0f * PWM_FREQ_HZ)) - 1);
        if (!writeReg(REG_PRESCALE, prescale)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // Wake up with auto-increment enabled
        if (!writeReg(REG_MODE1, MODE1_AI | MODE1_ALLCALL)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // All channels off
        for (int ch = 0; ch < 16; ++ch) setChannelDuty(ch, 0);
        return true;
    }

    void setChannelDuty(int channel, int duty) {
        duty = std::clamp(duty, 0, PWM_MAX);
        const uint8_t reg = static_cast<uint8_t>(
            REG_LED0_ON_L + channel * BYTES_PER_CH);
        uint8_t data[5];
        data[0] = reg;
        data[1] = 0x00;              // ON_L  (always start at 0)
        data[2] = 0x00;              // ON_H
        data[3] = duty & 0xFF;       // OFF_L
        data[4] = (duty >> 8) & 0x0F; // OFF_H
        if (write(i2cFd_, data, 5) != 5)
            std::cerr << "[MotorControl] I2C write failed ch " << channel << "\n";
    }

    bool writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = { reg, val };
        if (write(i2cFd_, buf, 2) != 2) {
            std::cerr << "[MotorControl] Failed to write reg 0x"
                      << std::hex << (int)reg << std::dec << "\n";
            return false;
        }
        return true;
    }

    bool initialised_ = false;
    int  i2cFd_       = -1;
    int  prevDuty_         [MOTOR_COUNT] = {};
    int  overdriveCounter_ [MOTOR_COUNT] = {};
};