// ============================================================
//  main.cpp
//  Entry point for the haptic feedback sleeve.
//  Raspberry Pi Zero 2 W + PCA9685 + ULN2803A + 12 ERM motors.
//
//  Dependency chain (each file includes the one above it):
//    AudioEngine.cpp  — DSP filtering         -> FilterOutput
//    Mappers.cpp      — BandMapper/DirectMapper -> MotorCommand
//    MotorControl.cpp — PCA9685 I2C PWM driver
//    Buttons.cpp      — GPIO button polling thread
//    main.cpp         — PortAudio capture, main loop, mode switching
//
//  Install dependencies:
//    sudo apt install libportaudio2 portaudio19-dev libi2c-dev
//    sudo raspi-config -> Interface Options -> I2C -> Enable
//    sudo usermod -aG gpio $USER   (then log out and back in)
//
//  Compile:
//    g++ -std=c++17 -O2 -o HapticSleeve main.cpp \
//        -li2c -lportaudio -lgpiod -lm
//
//  Run:
//    ./HapticSleeve
//
//  First-run checklist:
//    1. i2cdetect -y 1       <- confirm PCA9685 shows at 0x40
//    2. gpioinfo             <- confirm button pins are free
//    3. Check MOTOR_CHANNELS in MotorControl.cpp matches your PCB
//    4. Check PIN_* in Buttons.cpp matches your wiring
// ============================================================

#include "MotorControl.cpp"  // -> Mappers.cpp -> AudioEngine.cpp
#include "Buttons.cpp"

#include <portaudio.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <cstring>


// ─────────────────────────────────────────────────────────────
//  Audio config
//  256 samples @ 44100 Hz ~= 5.8 ms per callback.
//  Increase FRAMES_PER_BUFFER to 512 if you get audio xruns
//  on the Pi Zero (it has limited CPU compared to Pi 4).
// ─────────────────────────────────────────────────────────────

static constexpr int  FRAMES_PER_BUFFER  = 256;
static constexpr int  INPUT_DEVICE_INDEX = -1;    // -1 = system default
static constexpr bool LIST_DEVICES       = true;  // print mic list on startup


// ─────────────────────────────────────────────────────────────
//  System state
//
//  These atomics are the single source of truth for power,
//  volume, and mode. Buttons.cpp writes them; main loop reads.
//
//  current_function:
//    1 = BandMapper   (rows buzz together by frequency group)
//    2 = DirectMapper (one band per motor, proportional intensity)
// ─────────────────────────────────────────────────────────────

std::atomic<bool> system_on        { true };
std::atomic<int>  current_volume   { 50   };  // 0-100, starts at 50%
std::atomic<int>  current_function { 1    };  // starts in Band mode


// ─────────────────────────────────────────────────────────────
//  Mapper instances
//  Add new mappers here — MAPPER_TABLE and MAPPER_COUNT update
//  automatically. current_function - 1 = index into this table.
// ─────────────────────────────────────────────────────────────

static BandMapper   bandMapper;
static DirectMapper directMapper;

static IMapper* const MAPPER_TABLE[] = {
    &bandMapper,    // current_function == 1
    &directMapper,  // current_function == 2
};
static constexpr int MAPPER_COUNT =
    static_cast<int>(sizeof(MAPPER_TABLE) / sizeof(MAPPER_TABLE[0]));


// ─────────────────────────────────────────────────────────────
//  Shared audio state  (audio thread <-> main thread)
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
    // Number of lines in the status block — must match lines printed below
    static constexpr int STATUS_LINES = BAND_COUNT + 4;
    std::cout << "\033[" << STATUS_LINES << "A";

    // Header
    std::cout << "Mode:   " << std::left << std::setw(30)
              << mapper->name()
              << "Vol: " << std::setw(3) << current_volume.load() << "%"
              << "  Pwr: " << (system_on.load() ? "ON " : "OFF")
              << "          \n\n";

    // Per-band: audio envelope bar + motor intensity value
    for (int b = 0; b < BAND_COUNT; ++b) {
        const float audioEnv = out[b].envelope;
        const float motorInt = cmd.intensity[b];
        const int   filled   = static_cast<int>(
            std::clamp(audioEnv, 0.0f, 1.0f) * 20.0f);

        std::cout << std::left << std::setw(24)
                  << AudioEngine::bandName(b)
                  << " [" << std::string(filled, '#')
                           << std::string(20 - filled, '-') << "] "
                  << std::fixed << std::setprecision(3) << audioEnv
                  << "  m:" << std::setprecision(2) << motorInt
                  << "     \n";
    }

    // Motor intensity bar
    std::cout << "\n";
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

    // ── PCA9685 motor controller ──────────────────────────────
    MotorController motors;
    if (!motors.init()) return 1;

    // ── GPIO button handler ───────────────────────────────────
    ButtonManager buttons;
    if (!buttons.start()) {
        std::cerr << "[main] Button init failed — continuing without buttons.\n"
                  << "  Check PIN_* values in Buttons.cpp match your wiring.\n";
        // Non-fatal: system still works, just no physical buttons
    }

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

    std::cout << "Running — press buttons or Ctrl+C to stop\n\n";
    // Reserve lines for status display
    for (int i = 0; i < BAND_COUNT + 4; ++i) std::cout << "\n";

    // ── Main loop (~30 Hz) ────────────────────────────────────
    // cmd is persistent across frames — output smoothing in the
    // mappers needs memory between iterations.
    MotorCommand cmd;
    std::memset(cmd.intensity, 0, sizeof(cmd.intensity));

    while (gState.running.load(std::memory_order_relaxed)) {

        // Read latest FilterOutput from audio thread
        const int rs = 1 - gState.writeSlot.load(std::memory_order_acquire);
        const FilterOutput out = gState.slots[rs];

        // Select active mapper from current_function (set by mode button)
        const int funcIdx = std::clamp(
            current_function.load() - 1, 0, MAPPER_COUNT - 1);
        IMapper* activeMapper = MAPPER_TABLE[funcIdx];

        // Power gate — silence all motors when system is off
        if (!system_on.load()) {
            std::memset(cmd.intensity, 0, sizeof(cmd.intensity));
            motors.allOff();
            printStatus(out, cmd, motors, activeMapper);
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }

        // Run active mapper — volume (0-100) scales all intensities
        activeMapper->map(out, current_volume.load(), cmd);

        // Write PWM to PCA9685
        motors.update(cmd);

        // Refresh console
        printStatus(out, cmd, motors, activeMapper);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    // ── Shutdown ─────────────────────────────────────────────
    std::cout << "\nShutting down...\n";
    Pa_StopStream(stream);
    Pa_CloseStream(stream);
    Pa_Terminate();
    buttons.stop();
    motors.shutdown();

    return 0;
}