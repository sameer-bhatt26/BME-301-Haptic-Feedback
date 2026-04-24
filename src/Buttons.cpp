#pragma once

// ============================================================
//  Buttons.cpp
//  Physical button handling for the haptic feedback sleeve.
//
//  Four buttons are supported:
//    POWER_BTN     — toggle system on/off (all motors silent when off)
//    VOL_UP_BTN    — increase volume by VOLUME_STEP (max 100)
//    VOL_DOWN_BTN  — decrease volume by VOLUME_STEP (min 0)
//    MODE_BTN      — cycle between mapping modes (Band <-> Direct)
//
//  Each button is read on a dedicated polling thread so button
//  presses are never missed even if the main loop is busy.
//  Hardware debounce is implemented in software (DEBOUNCE_MS).
//
//  Wiring assumption:
//    Button connects GPIO pin to GND.
//    Internal pull-up is enabled — pin reads HIGH when idle,
//    LOW when button is pressed.
//    If your buttons pull HIGH instead, change ACTIVE_LEVEL below.
//
//  GPIO pin numbers (BCM numbering):
//    !! UPDATE THESE once you confirm your wiring !!
//    They are all together at the top for easy editing.
//
//  Uses libgpiod v2 (same library as the old MotorControl).
//  No sudo required — user must be in the gpio group.
// ============================================================

#include <gpiod.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <mutex>


// ─────────────────────────────────────────────────────────────
//  System state  (shared with main.cpp and MotorControl.cpp)
//  Declared extern — definitions live in main.cpp.
// ─────────────────────────────────────────────────────────────

extern std::atomic<bool> system_on;
extern std::atomic<int>  current_volume;
extern std::atomic<int>  current_function;


// ─────────────────────────────────────────────────────────────
//  Button GPIO pin assignments  (BCM numbers)
//
//  !! EDIT THESE to match your actual wiring !!
//  To find BCM numbers: run "pinout" on the Pi or check
//  https://pinout.xyz
// ─────────────────────────────────────────────────────────────

static constexpr int PIN_POWER     = 16;  // !! NOT BEING USED
static constexpr int PIN_VOL_UP    = 19;  // !! UPDATE
static constexpr int PIN_VOL_DOWN  = 6;  // !! UPDATE
static constexpr int PIN_MODE      = 20;  // !! CLOSEST TO THE BATTERY


// ─────────────────────────────────────────────────────────────
//  Button behaviour constants
//
//  VOLUME_STEP   — how much volume changes per button press (0-100)
//  VOLUME_MIN    — minimum volume (motors fully off)
//  VOLUME_MAX    — maximum volume
//  NUM_MODES     — total number of mapping modes (Band + Direct = 2)
//  DEBOUNCE_MS   — ignore re-triggers within this window (ms)
//  POLL_MS       — how often the button thread checks each pin (ms)
//                  Keep low (5-10 ms) so presses feel instant.
//  ACTIVE_LEVEL  — GPIO level when button IS pressed.
//                  0 = button pulls to GND (most common, pull-up wiring)
//                  1 = button pulls to VCC (pull-down wiring)
// ─────────────────────────────────────────────────────────────

static constexpr int  VOLUME_STEP  = 5;
static constexpr int  VOLUME_MIN   = 0;
static constexpr int  VOLUME_MAX   = 100;
static constexpr int  NUM_MODES    = 2;    // Band=1, Direct=2
static constexpr int  DEBOUNCE_MS  = 50;
static constexpr int  POLL_MS      = 5;
static constexpr int  ACTIVE_LEVEL = 0;   // 0 = active-low (pull-up)


// ─────────────────────────────────────────────────────────────
//  Thread-safe print helper
// ─────────────────────────────────────────────────────────────

static std::mutex g_printMutex;

static void safePrint(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_printMutex);
    std::cout << msg << "\n";
}


// ─────────────────────────────────────────────────────────────
//  Button actions
//  These functions are called from the button polling thread.
//  They only touch atomics — no GPIO, no blocking calls.
// ─────────────────────────────────────────────────────────────

static void onPower() {
    const bool newState = !system_on.load();
    system_on.store(newState);
    safePrint(newState ? "[Button] Power ON" : "[Button] Power OFF");
}

static void onVolumeUp() {
    const int vol = std::min(current_volume.load() + VOLUME_STEP, VOLUME_MAX);
    current_volume.store(vol);
    safePrint("[Button] Volume: " + std::to_string(vol) + "%");
}

static void onVolumeDown() {
    const int vol = std::max(current_volume.load() - VOLUME_STEP, VOLUME_MIN);
    current_volume.store(vol);
    if (vol == VOLUME_MIN)
        safePrint("[Button] Volume: MUTED");
    else
        safePrint("[Button] Volume: " + std::to_string(vol) + "%");
}

static void onMode() {
    // Cycle through modes 1..NUM_MODES, wrapping back to 1
    int next = current_function.load() + 1;
    if (next > NUM_MODES) next = 1;
    current_function.store(next);
    safePrint("[Button] Mode: " + std::to_string(next));
}


// ─────────────────────────────────────────────────────────────
//  ButtonManager
//  Opens all button GPIO lines and polls them on a background
//  thread.  Call start() once after main() initialises, and
//  stop() before shutdown.
// ─────────────────────────────────────────────────────────────

class ButtonManager {
public:

    bool start() {
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!chip_) {
            std::cerr << "[Buttons] Cannot open /dev/gpiochip0\n"
                      << "  sudo usermod -aG gpio $USER  (then re-login)\n";
            return false;
        }

        // Configure all four button lines as inputs with pull-ups
        const int pins[4] = { PIN_POWER, PIN_VOL_UP, PIN_VOL_DOWN, PIN_MODE };

        gpiod_line_settings* settings = gpiod_line_settings_new();
        if (!settings) { cleanup(); return false; }

        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);
        // Note: if your buttons pull HIGH instead of LOW, change to:
        // gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_DOWN);

        gpiod_line_config* lineCfg = gpiod_line_config_new();
        if (!lineCfg) {
            gpiod_line_settings_free(settings);
            cleanup(); return false;
        }

        for (int pin : pins) {
            const unsigned int offset = static_cast<unsigned int>(pin);
            gpiod_line_config_add_line_settings(lineCfg, &offset, 1, settings);
        }

        gpiod_request_config* reqCfg = gpiod_request_config_new();
        if (reqCfg) gpiod_request_config_set_consumer(reqCfg, "haptic-buttons");

        request_ = gpiod_chip_request_lines(chip_, reqCfg, lineCfg);

        gpiod_line_settings_free(settings);
        gpiod_line_config_free(lineCfg);
        if (reqCfg) gpiod_request_config_free(reqCfg);

        if (!request_) {
            std::cerr << "[Buttons] Failed to request GPIO lines.\n"
                      << "  Check pins " << PIN_POWER << " " << PIN_VOL_UP
                      << " " << PIN_VOL_DOWN << " " << PIN_MODE
                      << " aren't already in use.\n"
                      << "  Run: gpioinfo | grep -E 'line|offset'\n";
            cleanup(); return false;
        }

        running_ = true;
        thread_  = std::thread(&ButtonManager::pollLoop, this);

        std::cout << "[Buttons] Ready. Pins BCM: "
                  << "power=" << PIN_POWER
                  << " vol+=" << PIN_VOL_UP
                  << " vol-=" << PIN_VOL_DOWN
                  << " mode=" << PIN_MODE << "\n";
        return true;
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        cleanup();
        std::cout << "[Buttons] Stopped.\n";
    }

    ~ButtonManager() { stop(); }


private:

    // ── Polling loop — runs on its own thread ────────────────

    void pollLoop() {
        using Clock = std::chrono::steady_clock;

        // Track state and last-trigger time per button
        // Order: power, vol_up, vol_down, mode
        const unsigned int offsets[4] = {
            static_cast<unsigned int>(PIN_POWER),
            static_cast<unsigned int>(PIN_VOL_UP),
            static_cast<unsigned int>(PIN_VOL_DOWN),
            static_cast<unsigned int>(PIN_MODE),
        };

        int  prevLevel[4] = { 1, 1, 1, 1 };  // idle = HIGH (pull-up)
        std::chrono::steady_clock::time_point lastTrigger[4] = { Clock::now(), Clock::now(),
                                Clock::now(), Clock::now() };

        while (running_) {
            for (int i = 0; i < 4; ++i) {
                const gpiod_line_value raw =
                    gpiod_line_request_get_value(request_, offsets[i]);
                const int level = (raw == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;

                // Detect falling edge (idle->pressed transition)
                const bool pressed = (level == ACTIVE_LEVEL) &&
                                     (prevLevel[i] != ACTIVE_LEVEL);

                if (pressed) {
                    const auto now     = Clock::now();
                    const auto elapsed = std::chrono::duration_cast<
                        std::chrono::milliseconds>(now - lastTrigger[i]).count();

                    if (elapsed >= DEBOUNCE_MS) {
                        lastTrigger[i] = now;
                        // Dispatch to the appropriate action
                        switch (i) {
                            case 0: onPower();      break;
                            case 1: onVolumeUp();   break;
                            case 2: onVolumeDown(); break;
                            case 3: onMode();       break;
                        }
                    }
                }

                prevLevel[i] = level;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
        }
    }

    // ── Cleanup ──────────────────────────────────────────────

    void cleanup() {
        if (request_) { gpiod_line_request_release(request_); request_ = nullptr; }
        if (chip_)    { gpiod_chip_close(chip_);              chip_    = nullptr; }
    }

    gpiod_chip*         chip_    = nullptr;
    gpiod_line_request* request_ = nullptr;
    std::atomic<bool>   running_ { false };
    std::thread         thread_;
};