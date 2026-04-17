// ============================================================
//  main.cpp
//  Entry point for the haptic feedback sleeve on Raspberry Pi.
//
//  Dependency chain (each file includes the one above it):
//    AudioEngine.cpp   — DSP filtering, produces FilterOutput
//    Mappers.cpp       — IMapper interface + BandMapper, DirectMapper,
//                        DirectionalMapper (stub)
//    MotorControl.cpp  — libgpiod GPIO driver, takes MotorCommand
//    main.cpp          — PortAudio capture, threading, mode switching
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp -lgpiod -lportaudio -lm
//
//  Run:
//    ./HapticSleeve
// ============================================================

#include "MotorControl.cpp"   // pulls in Mappers.cpp -> AudioEngine.cpp

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

// Samples per PortAudio callback buffer.
// 256 @ 44100 Hz ~= 5.8 ms.  Increase to 512 if you get xruns.
static constexpr int  FRAMES_PER_BUFFER  = 256;

// -1 = system default input device.
// Change to a specific index if your USB mic is not the default.
// Set LIST_DEVICES = true on first run to see available indices.
static constexpr int  INPUT_DEVICE_INDEX = -1;
static constexpr bool LIST_DEVICES       = true;


// ─────────────────────────────────────────────────────────────
//  System state
//  Written by button handlers (VolumeAndFunctionButtons.cpp)
//  and read by the main loop.  All atomic — no mutex needed.
//
//  system_on        — power state; false = motors off, skip mapping
//  current_volume   — 0-100, scales envelope inside every mapper
//  current_function — 1=BandMapper  2=DirectMapper  3=Directional
//                     must match switch_function() in button file
// ─────────────────────────────────────────────────────────────

// When VolumeAndFunctionButtons.cpp is linked in, remove these
// definitions and replace with:
//   extern std::atomic<bool> system_on;
//   extern std::atomic<int>  current_volume;
//   extern std::atomic<int>  current_function;
std::atomic<bool> system_on        { true };
std::atomic<int>  current_volume   { 50   };  // start at 50%
std::atomic<int>  current_function { 1    };  // start in BandMapper mode


// ─────────────────────────────────────────────────────────────
//  Mapper instances
//  One of each mode, selected at runtime via MAPPER_TABLE.
//  To add a new mode: construct it here, add to MAPPER_TABLE,
//  and increment the function button cycle in switch_function().
// ─────────────────────────────────────────────────────────────

static BandMapper        bandMapper;
static DirectMapper      directMapper;
static DirectionalMapper directionalMapper;

// Index = current_function - 1
static IMapper* const MAPPER_TABLE[] = {
    &bandMapper,           // function 1
    &directMapper,         // function 2
    &directionalMapper,    // function 3
};
static constexpr int MAPPER_COUNT =
    static_cast<int>(sizeof(MAPPER_TABLE) / sizeof(MAPPER_TABLE[0]));


// ─────────────────────────────────────────────────────────────
//  Shared state  (audio thread <-> main thread)
//  Lock-free ping-pong buffer — no mutexes in the audio path.
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
                        const MotorController& motors,
                        const IMapper*         mapper)
{
    std::cout << "\033[" << (BAND_COUNT + 4) << "A";

    std::cout << "Mode:   " << mapper->name() << "          \n";
    std::cout << "Volume: " << current_volume.load()
              << "%   Power: " << (system_on.load() ? "ON " : "OFF")
              << "          \n\n";

    for (int b = 0; b < BAND_COUNT; ++b) {
        const float e      = out[b].envelope;
        const int   filled = static_cast<int>(
            std::clamp(e, 0.0f, 1.0f) * 20.0f);
        std::cout << std::left  << std::setw(26)
                  << AudioEngine::bandName(b)
                  << " [" << std::string(filled, '#')
                           << std::string(20 - filled, '-') << "] "
                  << std::fixed << std::setprecision(3) << e
                  << "          \n";
    }

    motors.printState();
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

    // ── GPIO ─────────────────────────────────────────────────
    MotorController motors;
    if (!motors.init()) return 1;

    // ── PortAudio ─────────────────────────────────────────────
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
    while (gState.running.load(std::memory_order_relaxed)) {

        // Read latest FilterOutput from the audio thread
        const int rs = 1 - gState.writeSlot.load(std::memory_order_acquire);
        const FilterOutput out = gState.slots[rs];

        // Select active mapper from current_function
        // (written by switch_function() in VolumeAndFunctionButtons.cpp)
        const int funcIdx = std::clamp(
            current_function.load() - 1, 0, MAPPER_COUNT - 1);
        IMapper* activeMapper = MAPPER_TABLE[funcIdx];

        // Power gate — if off, silence motors and skip mapping
        if (!system_on.load()) {
            motors.allOff();
            printStatus(out, motors, activeMapper);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        // Run the active mapper
        // current_volume (0-100) is passed so mappers can scale
        // envelope thresholds without touching AudioEngine
        MotorCommand cmd;
        activeMapper->map(out, current_volume.load(), cmd);

        // Write GPIO
        motors.update(cmd);

        // Refresh console
        printStatus(out, motors, activeMapper);

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