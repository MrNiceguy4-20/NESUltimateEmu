#pragma once
#include <cstdint>
#include <vector>
#include <mutex>
#include <array>

class Bus;
class CPU;
class Cartridge;

class APU {
public:
    APU();
    ~APU();

    void connectBus(Bus* bus) { m_bus = bus; }
    void connectCPU(CPU* cpu) { m_cpu = cpu; }
    void connectCartridge(Cartridge* cart) { m_cart = cart; }

    void cpuWrite(uint16_t addr, uint8_t data);
    uint8_t cpuRead(uint16_t addr) const;

    void reset();
    void clock();
    void clockFrameCounterPhase();
    void fillBuffer(float* stream, int len);

    bool irqActive() const { return m_frameIrqFlag || m_dmc.irqFlag; }
    void completeDmcDma(uint8_t data);

    struct ChannelDebug {
        bool enabled = false;
        uint16_t timer = 0;
        uint16_t period = 0;
        uint8_t length = 0;
        uint8_t volume = 0;
    };

    struct DebugState {
        ChannelDebug pulse1;
        ChannelDebug pulse2;
        ChannelDebug triangle;
        ChannelDebug noise;
        ChannelDebug dmc;
        uint8_t triangleLinear = 0;
        uint16_t noiseShift = 0;
        uint16_t dmcAddress = 0;
        uint16_t dmcBytesRemaining = 0;
        uint8_t dmcBitsRemaining = 0;
        bool dmcDmaPending = false;
        bool frameMode5 = false;
        bool irqInhibit = false;
        bool frameIrq = false;
        bool dmcIrq = false;
        uint32_t frameCycles = 0;
        uint8_t frameJitter = 0;
    };

    DebugState debugState() const;
    uint8_t debugStatus() const;

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

    bool initAudio();
    void shutdownAudio();

    void setChipMod(bool enabled);
    bool chipMod() const { return m_chipMod; }

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

        void clockTimer();
        void start();
        float sample() const;
    };

    Pulse m_pulse1, m_pulse2;
    Triangle m_triangle;
    Noise m_noise;
    Dmc m_dmc;

    bool m_frameMode5 = false;
    bool m_irqInhibit = false;
    mutable bool m_frameIrqFlag = false;
    uint32_t m_frameCycles = 0;
    bool m_apuPhase = false;
    uint8_t m_frameJitter = 0;
    bool m_frameWriteThisCycle = false;

    bool m_chipMod = false;

    static constexpr int kSampleRate = 44100;
    static constexpr int kCpuClock = 1789773;
    int m_outputSampleRate = kSampleRate;
    bool m_audioOpen = false;
    uint32_t m_audioDeviceId = 0;
    double m_sampleTimer = 0;
    double m_samplePeriod = (double)kCpuClock / kSampleRate;
    double m_mixAccumulator = 0.0;
    double m_mixAccumulatorWeight = 0.0;
    double m_hpPrevInput = 0.0;
    double m_hpPrevOutput = 0.0;
    double m_hpCoefficient = 0.0;
    std::array<float, 2048> m_triangleGain{};

    std::mutex m_mutex;
    std::vector<float> m_ring;
    size_t m_ringWrite = 0;
    size_t m_ringRead = 0;
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
    static const uint16_t noisePeriods[16];
    static const uint8_t triangleSequence[32];
    static const uint16_t dmcRates[16];
};
