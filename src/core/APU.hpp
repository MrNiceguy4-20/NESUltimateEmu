#pragma once
#include <cstdint>
#include <vector>
#include <atomic>
#include <array>
#include "Timing.hpp"

class Bus;
class CPU;
class Cartridge;

class APU {
public:
    enum class DmcCpuRevision : uint8_t {
        PreMid1990 = 0,
        Mid1990OrLater = 1,
    };

    APU();
    ~APU();

    void connectBus(Bus* bus) { m_bus = bus; }
    void connectCPU(CPU* cpu) { m_cpu = cpu; }
    void connectCartridge(Cartridge* cart) { m_cart = cart; }

    void cpuWrite(uint16_t addr, uint8_t data);
    uint8_t cpuRead(uint16_t addr) const;

    void powerOn();
    void reset();
    void clock();
    void clockFrameCounterPreCpuPhase();
    void clockFrameCounterPhase();
    void setTiming(ConsoleTiming timing);
    ConsoleTiming timing() const { return m_timing; }
    int cpuClockHz() const { return consoleCpuClockHz(m_timing); }
    void fillBuffer(float* stream, int len);

    bool irqActive() const { return (m_frameIrqFlag && !m_irqInhibit) || m_dmc.irqFlag; }
    void completeDmcDma(uint8_t data);
    void abortDmcDma();

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

    bool initAudio();
    void shutdownAudio();

    void setChipMod(bool enabled);
    bool chipMod() const { return m_chipMod; }
    void setMasterVolume(float volume);
    float masterVolume() const { return m_masterVolume; }

    size_t queuedAudioSamples();
    size_t audioBufferCapacity() const { return kRingSize - 1; }
    uint64_t audioUnderrunCount() const;
    uint64_t audioOverrunCount() const;
    int outputSampleRate() const { return m_outputSampleRate; }
    bool audioOpen() const { return m_audioOpen; }
    void setHostAudioEnabled(bool enabled);
    void setAudioPlaybackPaused(bool paused);
    bool audioPlaybackPaused() const { return m_audioPlaybackPaused; }
    bool hostAudioEnabled() const { return m_hostAudioEnabled; }

    void setDmcCpuRevision(DmcCpuRevision revision) { m_dmcCpuRevision = revision; }
    DmcCpuRevision dmcCpuRevision() const { return m_dmcCpuRevision; }

#ifdef NES_HEADLESS

    bool testFrameIrqFlag() const { return m_frameIrqFlag; }
    void testSetFrameIrqFlag(bool set) { m_frameIrqFlag = set; }
    bool testFrameMode5() const { return m_frameMode5; }
    uint32_t testFrameCycles() const { return m_frameCycles; }
    uint8_t testFrameResetDelay() const { return m_frameResetDelay; }
    uint8_t testPulse1Length() const { return m_pulse1.length; }
    uint16_t testNoisePeriod() const { return m_noise.timerPeriod; }
    uint16_t testDmcRate() const { return m_dmc.rate; }
    void testSetPulse1Length(uint8_t length) { m_pulse1.length = length; m_pulse1.enabled = true; }
    void testPrimeOneByteDmcLoad(uint16_t address, uint16_t timer) {
        m_dmc.enabled = true;
        m_dmc.loop = false;
        m_dmc.sampleLength = 1;
        m_dmc.currentAddr = address;
        m_dmc.bytesRemaining = 1;
        m_dmc.sampleBufferFull = false;
        m_dmc.bitsRemaining = 0;
        m_dmc.timer = timer;
        m_dmc.dmaPending = true;
        m_dmc.dmaLoadPending = false;
        m_dmc.dmaAbortPending = false;
        m_dmc.dmaImplicitAbortPending = false;
        m_dmc.dmaForcedReloadPending = false;
        m_dmc.implicitStopWindow = 0;
    }
    bool testDmcAbortPending() const { return m_dmc.dmaAbortPending; }
    bool testDmcForcedReloadPending() const { return m_dmc.dmaForcedReloadPending; }
    uint16_t testDmcCurrentAddr() const { return m_dmc.currentAddr; }
#endif

private:
    Bus* m_bus = nullptr;
    CPU* m_cpu = nullptr;
    Cartridge* m_cart = nullptr;

    struct Pulse {
        bool enabled = false;
        uint8_t duty = 0;
        uint8_t volume = 0;
        bool constant = false;
        bool lengthHalt = false;
        bool pendingLengthHaltValid = false;
        bool pendingLengthHalt = false;
        bool pendingLengthReload = false;
        uint8_t pendingLengthValue = 0;
        uint16_t timer = 0;
        uint16_t timerPeriod = 0;
        uint8_t length = 0;
        uint8_t dutyBit = 0;
        uint8_t envelope = 0;
        uint8_t envelopePeriod = 0;
        bool envelopeStart = false;
        uint8_t sweepShift = 0;
        bool sweepNegate = false;
        uint8_t sweepPeriod = 0;
        bool sweepEnabled = false;
        bool sweepReload = false;
        uint8_t sweepDivider = 0;

        void clockTimer();
        void clockEnvelope();
        void clockLength();
        void clockSweep(bool isPulse1);
        int sweepTarget(bool isPulse1) const;
        bool muted(bool isPulse1) const;
        float sample(bool isPulse1) const;
    };

    struct Triangle {
        bool enabled = false;
        bool lengthHalt = false;
        bool pendingLengthHaltValid = false;
        bool pendingLengthHalt = false;
        bool pendingLengthReload = false;
        uint8_t pendingLengthValue = 0;
        uint16_t timer = 0;
        uint16_t timerPeriod = 0;
        uint8_t length = 0;
        uint8_t linear = 0;
        uint8_t linearReload = 0;
        bool linearReloadFlag = false;
        uint8_t sequencer = 0;
        float phase = 0.0f;

        void clockTimer();
        void clockLinear();
        void clockLength();
        float sample(bool chipMod) const;
    };

    struct Noise {
        bool enabled = false;
        bool lengthHalt = false;
        bool pendingLengthHaltValid = false;
        bool pendingLengthHalt = false;
        bool pendingLengthReload = false;
        uint8_t pendingLengthValue = 0;
        bool constant = false;
        uint8_t volume = 0;
        uint8_t envelope = 0;
        uint8_t envelopePeriod = 0;
        bool envelopeStart = false;
        uint16_t timer = 0;
        uint16_t timerPeriod = 0;
        uint8_t length = 0;
        uint16_t shift = 1;
        bool mode = false;
        float smooth = 0.0f;

        void clockTimer();
        void clockEnvelope();
        void clockLength();
        float sample() const;
    };

    struct Dmc {
        bool enabled = false;
        bool irqEnabled = false;
        bool loop = false;
        mutable bool irqFlag = false;
        uint16_t rate = 428;
        uint16_t timer = 0;
        uint8_t output = 0;
        uint8_t sampleBuffer = 0;
        bool sampleBufferFull = false;
        uint8_t shiftReg = 0;
        uint8_t bitsRemaining = 0;
        bool silence = true;
        uint16_t sampleAddr = 0xC000;
        uint16_t sampleLength = 1;
        uint16_t currentAddr = 0xC000;
        uint16_t bytesRemaining = 0;
        bool dmaPending = false;

        uint8_t dmaStartDelay = 0;
        bool dmaLoadPending = false;

        bool dmaReloadWaitingForEnable = false;

        uint8_t stopBugWindow = 0;
        uint8_t implicitStopWindow = 0;
        bool dmaAbortPending = false;

        bool dmaImplicitAbortPending = false;

        uint8_t dmaImplicitAbortDelay = 0;

        bool dmaForcedReloadPending = false;

        void clockTimer();
        void start();
        float sample() const;
    };

    Pulse m_pulse1, m_pulse2;
    Triangle m_triangle;
    Noise m_noise;
    Dmc m_dmc;

    DmcCpuRevision m_dmcCpuRevision = DmcCpuRevision::Mid1990OrLater;

    bool m_frameMode5 = false;

    bool m_frameResetAppliedPreCpu = false;
    bool m_irqInhibit = false;
    mutable bool m_frameIrqFlag = false;
    mutable bool m_frameIrqClearPending = false;
    uint32_t m_frameCycles = 0;
    bool m_apuPhase = false;

    uint8_t m_frameResetDelay = 0;

    uint8_t m_pendingFrameStartCycles = 0;
    bool m_pendingFrameMode5 = false;

    bool m_chipMod = false;
    ConsoleTiming m_timing = ConsoleTiming::NTSC;

    static constexpr int kSampleRate = 44100;
    int m_outputSampleRate = kSampleRate;
    bool m_audioOpen = false;
    float m_masterVolume = 1.50f;
    uint32_t m_audioDeviceId = 0;
    bool m_hostAudioEnabled = true;
    std::atomic<uint64_t> m_audioUnderruns{0};
    std::atomic<uint64_t> m_audioOverruns{0};
    double m_sampleTimer = 0;
    double m_samplePeriod = (double)consoleCpuClockHz(ConsoleTiming::NTSC) / kSampleRate;
    double m_mixAccumulator = 0.0;
    double m_mixAccumulatorWeight = 0.0;

    double m_hp90PrevInput = 0.0;
    double m_hp90PrevOutput = 0.0;
    double m_hp440PrevInput = 0.0;
    double m_hp440PrevOutput = 0.0;
    double m_lp14kPrevOutput = 0.0;
    double m_hp90Coefficient = 0.0;
    double m_hp440Coefficient = 0.0;
    double m_lp14kCoefficient = 0.0;
    std::array<float, 2048> m_triangleGain{};

    std::vector<float> m_ring;
    std::atomic<size_t> m_ringWrite{0};
    std::atomic<size_t> m_ringRead{0};
    bool m_audioPlaybackPaused = true;
    static constexpr size_t kRingSize = 8192;

    void clockFrameCounter();
    void applyPendingLengthWrites();
    void scheduleDmcDma();
    void quarterFrame();
    void halfFrame();
    void pushSample(float s);
    float mixInstantSample() const;
    float filterOutput(float s);
    void resetOutputPipeline();
    void rebuildTriangleLoudnessTable();
    float triangleLoudnessGain(uint16_t period) const;

    static const uint8_t lengthTable[32];
    static const uint8_t dutyTable[4][8];
    static const uint16_t noisePeriodsNtsc[16];
    static const uint16_t noisePeriodsPal[16];
    static const uint8_t triangleSequence[32];
    static const uint16_t dmcRatesNtsc[16];
    static const uint16_t dmcRatesPal[16];
};
