// ============================================================
//  main.cpp
//  Entry point for the haptic feedback sleeve on Raspberry Pi Zero 2 W.
//
//  Dependency chain (each file includes the one above it):
//    AudioEngine.cpp  — DSP filtering        -> FilterOutput
//    Mappers.cpp      — mode logic            -> MotorCommand (float intensity)
//    MotorControl.cpp — PCA9685 I2C PWM driver
//    main.cpp         — PortAudio capture, threading, mode switching
//
//  Install dependencies on Pi:
//    sudo apt install libportaudio2 portaudio19-dev libi2c-dev
//    sudo raspi-config -> Interface Options -> I2C -> Enable
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -li2c -lportaudio -lm
//
//  Run:
//    ./HapticSleeve
// ============================================================

#include "MotorControl.cpp"  // pulls in Mappers.cpp -> AudioEngine.cpp

#include <portaudio.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>


// ─────────────────────────────────────────────────────────────
//  Audio capture config
// ─────────────────────────────────────────────────────────────

// Samples per PortAudio callback.  256 @ 44100 Hz ~= 5.8 ms.
// Increase to 512 if you get xruns on the Pi Zero.
static constexpr int  FRAMES_PER_BUFFER  = 256;

// -1 = system default input device.
// Run once with LIST_DEVICES = true to find your mic index,
// then set INPUT_DEVICE_INDEX to that number.
static constexpr int  INPUT_DEVICE_INDEX = -1;
static constexpr bool LIST_DEVICES       = true;


// ─────────────────────────────────────────────────────────────
//  System state  (written by button handlers, read by main loop)
//
//  These are defined here so the rest of the code can see them.
//  When VolumeAndFunctionButtons.cpp is wired in:
//    1. Remove the three definitions below.
//    2. Add at the top of this file:
//         extern std::atomic<bool> system_on;
//         extern std::atomic<int>  current_volume;
//         extern std::atomic<int>  current_function;
//    3. The button file already defines them — no other changes needed.
//
//  current_function mapping:
//    1 = BandMapper      (rows buzz together by frequency group)
//    2 = DirectMapper    (one band per motor, proportional intensity)
//    3 = DirectionalMapper (two-mic ILD — falls back to Band until 2nd mic added)
// ─────────────────────────────────────────────────────────────

std::atomic<bool> system_on        { true };
std::atomic<int>  current_volume   { 50   };  // 0-100
std::atomic<int>  current_function { 1    };  // start in BandMapper


// ─────────────────────────────────────────────────────────────
//  Mapper instances and selection table
//  Add new mappers here — no other files need changing.
// ─────────────────────────────────────────────────────────────

static BandMapper        bandMapper;
static DirectMapper      directMapper;
static DirectionalMapper directionalMapper;

static IMapper* const MAPPER_TABLE[] = {
    &bandMapper,           // current_function == 1
    &directMapper,         // current_function == 2
    &directionalMapper,    // current_function == 3
};
static constexpr int MAPPER_COUNT =
    static_cast<int>(sizeof(MAPPER_TABLE) / sizeof(MAPPER_TABLE[0]));


// ─────────────────────────────────────────────────────────────
//  Shared state  (audio thread <-> main thread)
//  Lock-free ping-pong — no mutexes in the audio callback.
// ─────────────────────────────────────────────────────────────

struct SharedState {
    FilterOutput      slots[2];
    std::atomic<int>  writeSlot { 0 };
    std::atomic<bool> running   { true };
};

static SharedState gState;


// ─────────────────────────────────────────────────────────────
//  PortAudio callback  (audio thread)
//  Rules: no heap alloc, no blocking calls, no std::cout.
// ─────────────────────────────────────────────────────────────

struct CallbackData { AudioEngine engine; };

static int audioCallback(const void*                     inputBuffer,
                         void*                           /* outputBuffer */,
                         unsigned long                   framesPerBuffer,
                         const PaStreamCallbackTimeInfo* /* timeInfo */,
                         PaStreamCallbackFlags           /* statusFlags */,
                         void*                           userData)
{
    auto* cb  = static_cast<CallbackData*>(userData);
    auto* pcm = static_cast<const int16_t*>(inputBuffer);
    if (!pcm) return paContinue;

    float buf[FRAMES_PER_BUFFER];
    for (unsigned long i = 0; i < framesPerBuffer; ++i)
        buf[i] = pcm[i] / 32768.0f;

    const std::vector<float> samples(buf, buf + framesPerBuffer);
    FilterOutput result = cb->engine.process(samples);

    const int ws = gState.writeSlot.load(std::memory_order_relaxed);
    gState.slots[ws] = result;
    gState.writeSlot.store(1 - ws, std::memory_order_release);

    return gState.running.load(std::memory_order_relaxed)
               ? paContinue : paComplete;
}


// ─────────────────────────────────────────────────────────────
//  Console helpers
// ─────────────────────────────────────────────────────────────

static void listDevices() {
    std::cout << "\nAvailable input devices:\n";
    for (int i = 0; i < Pa_GetDeviceCount(); ++i) {
        const PaDeviceInfo* info = Pa_GetDeviceInfo(i);
        if (info && info->maxInputChannels > 0)
            std::cout << "  [" << i << "] " << info->name << "\n";
    }
    std::cout << "  Default: [" << Pa_GetDefaultInputDevice() << "]\n\n";
}

static void printStatus(const FilterOutput&    out,
                        const MotorCommand&    cmd,
                        const MotorController& motors,
                        const IMapper*         mapper)
{
    std::cout << "\033[" << (BAND_COUNT + 4) << "A";

    std::cout << "Mode:   " << mapper->name()
              << "                    \n";
    std::cout << "Volume: " << current_volume.load()
              << "%   Power: " << (system_on.load() ? "ON " : "OFF")
              << "                    \n\n";

    for (int b = 0; b < BAND_COUNT; ++b) {
        const float audioEnv = out[b].envelope;
        const float motorInt = cmd.intensity[b];
        const int   filled   = static_cast<int>(
            std::clamp(audioEnv, 0.0f, 1.0f) * 20.0f);

        std::cout << std::left  << std::setw(26)
                  << AudioEngine::bandName(b)
                  << " audio["
                  << std::string(filled, '#')
                  << std::string(20 - filled, '-')
                  << "] "
                  << std::fixed << std::setprecision(3) << audioEnv
                  << "  motor "
                  << std::fixed << std::setprecision(3) << motorInt
                  << "          \n";
    }

    motors.printState(cmd);
    std::cout.flush();
}

static void onSignal(int) {
    gState.running.store(false, std::memory_order_relaxed);
}


// ─────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────

int main() {
    std::signal(SIGINT,  onSignal);
    std::signal(SIGTERM, onSignal);

    // ── PCA9685 init ─────────────────────────────────────────
    MotorController motors;
    if (!motors.init()) return 1;

    // ── PortAudio init ────────────────────────────────────────
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        std::cerr << "PortAudio init failed: " << Pa_GetErrorText(err) << "\n";
        return 1;
    }
    if (LIST_DEVICES) listDevices();

    const PaDeviceIndex devIdx = (INPUT_DEVICE_INDEX < 0)
                                     ? Pa_GetDefaultInputDevice()
                                     : INPUT_DEVICE_INDEX;
    const PaDeviceInfo* devInfo = Pa_GetDeviceInfo(devIdx);
    if (!devInfo) {
        std::cerr << "Invalid device index " << devIdx << "\n";
        Pa_Terminate(); return 1;
    }

    std::cout << "Mic:    [" << devIdx << "] " << devInfo->name << "\n";
    std::cout << "Buffer: " << FRAMES_PER_BUFFER << " samples (~"
              << std::fixed << std::setprecision(1)
              << 1000.0f * FRAMES_PER_BUFFER / SAMPLE_RATE << " ms)\n\n";

    PaStreamParameters params;
    params.device                    = devIdx;
    params.channelCount              = 1;
    params.sampleFormat              = paInt16;
    params.suggestedLatency          = devInfo->defaultLowInputLatency;
    params.hostApiSpecificStreamInfo = nullptr;

    CallbackData cbData;
    PaStream*    stream = nullptr;

    err = Pa_OpenStream(&stream, &params, nullptr,
                        SAMPLE_RATE, FRAMES_PER_BUFFER,
                        paClipOff, audioCallback, &cbData);
    if (err != paNoError) {
        std::cerr << "Pa_OpenStream: " << Pa_GetErrorText(err) << "\n";
        Pa_Terminate(); return 1;
    }

    err = Pa_StartStream(stream);
    if (err != paNoError) {
        std::cerr << "Pa_StartStream: " << Pa_GetErrorText(err) << "\n";
        Pa_CloseStream(stream); Pa_Terminate(); return 1;
    }

    std::cout << "Running — Ctrl+C to stop\n\n";
    for (int i = 0; i < BAND_COUNT + 4; ++i) std::cout << "\n";

    // ── Main loop (~30 Hz) ────────────────────────────────────
    MotorCommand cmd;  // persisted across frames for output smoothing

    while (gState.running.load(std::memory_order_relaxed)) {

        // Read latest FilterOutput from audio thread
        const int rs = 1 - gState.writeSlot.load(std::memory_order_acquire);
        const FilterOutput out = gState.slots[rs];

        // Select active mapper
        const int funcIdx = std::clamp(
            current_function.load() - 1, 0, MAPPER_COUNT - 1);
        IMapper* activeMapper = MAPPER_TABLE[funcIdx];

        // Power gate — silence everything when system is off
        if (!system_on.load()) {
            std::memset(cmd.intensity, 0, sizeof(cmd.intensity));
            motors.allOff();
            printStatus(out, cmd, motors, activeMapper);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        // Run the active mapper
        // volume (0-100) scales all intensities proportionally
        activeMapper->map(out, current_volume.load(), cmd);

        // Write PWM values to PCA9685
        motors.update(cmd);

        // Refresh console display
        printStatus(out, cmd, motors, activeMapper);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // ── Shutdown ─────────────────────────────────────────────
    std::cout << "\nShutting down...\n";
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    motors.shutdown();

    return 0;
}