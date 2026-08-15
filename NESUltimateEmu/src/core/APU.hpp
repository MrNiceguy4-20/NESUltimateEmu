#pragma once
#include <cstdint>
#include <vector>
#include <mutex>

class Bus;

class APU {
public:
    APU();
    ~APU();

    void connectBus(Bus* bus) { m_bus = bus; }

    void cpuWrite(uint16_t addr, uint8_t data);
    uint8_t cpuRead(uint16_t addr) const;

    void clock();
    void fillBuffer(float* stream, int len);

    void saveState(std::vector<uint8_t>& out) const;
    bool loadState(const uint8_t*& p, const uint8_t* end);

    bool initAudio();
    void shutdownAudio();

    void setChipMod(bool enabled) { m_chipMod = enabled; }
    bool chipMod() const { return m_chipMod; }

private:
    Bus* m_bus = nullptr;

    struct Pulse {
        bool enabled = false;
        uint8_t duty = 0;
        uint8_t volume = 0;
        bool constant = false;
        bool lengthHalt = false;
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
        float sample() const;
    };

    struct Triangle {
        bool enabled = false;
        bool lengthHalt = false;
        uint16_t timer = 0;
        uint16_t timerPeriod = 0;
        uint8_t length = 0;
        uint8_t linear = 0;
        uint8_t linearReload = 0;
        bool linearReloadFlag = false;
        uint8_t sequencer = 0;
        float phase = 0.0f;

        void clockTimer(bool chipMod);
        void clockLinear();
        void clockLength();
        float sample(bool chipMod) const;
    };

    struct Noise {
        bool enabled = false;
        bool lengthHalt = false;
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

        void clockTimer(bool chipMod);
        void clockEnvelope();
        void clockLength();
        float sample(bool chipMod) const;
    };

    // DMC (Delta Modulation Channel)
    struct Dmc {
        bool enabled = false;
        bool irqEnabled = false;
        bool loop = false;
        uint16_t rate = 0;
        uint16_t timer = 0;
        uint8_t output = 0;       // 0-127
        uint8_t sampleBuffer = 0;
        bool sampleBufferFull = false;
        uint8_t shiftReg = 0;
        uint8_t bitsRemaining = 0;
        bool silence = true;
        uint16_t sampleAddr = 0;
        uint16_t sampleLength = 0;
        uint16_t currentAddr = 0;
        uint16_t bytesRemaining = 0;

        void clockTimer(Bus* bus);
        void start();
        float sample() const;
    };

    Pulse m_pulse1, m_pulse2;
    Triangle m_triangle;
    Noise m_noise;
    Dmc m_dmc;

    bool m_frameMode5 = false;
    bool m_irqInhibit = false;
    uint32_t m_frameCycles = 0;
    bool m_chipMod = false;

    static constexpr int kSampleRate = 44100;
    static constexpr int kCpuClock = 1789773;
    double m_sampleTimer = 0;
    double m_samplePeriod = (double)kCpuClock / kSampleRate;

    std::mutex m_mutex;
    std::vector<float> m_ring;
    size_t m_ringWrite = 0;
    size_t m_ringRead = 0;
    static constexpr size_t kRingSize = 8192;

    void clockFrameCounter();
    void quarterFrame();
    void halfFrame();
    void pushSample(float s);
    float mixSample() const;
    float triangleLoudnessGain(uint16_t period) const;

    static const uint8_t lengthTable[32];
    static const uint8_t dutyTable[4][8];
    static const uint16_t noisePeriods[16];
    static const uint8_t triangleSequence[32];
    static const uint16_t dmcRates[16];
};
