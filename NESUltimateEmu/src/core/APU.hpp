#pragma once
#include <cstdint>
#include <atomic>
#include <vector>
#include <mutex>

class APU {
public:
    APU();
    ~APU();

    void cpuWrite(uint16_t addr, uint8_t data);
    uint8_t cpuRead(uint16_t addr);

    // Called once per CPU cycle
    void clock();

    // SDL audio callback fills stream with samples
    void fillBuffer(float* stream, int len);

    bool initAudio();
    void shutdownAudio();

private:
    // Pulse channel
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
        uint16_t sweepTarget = 0;

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

        void clockTimer();
        void clockLinear();
        void clockLength();
        float sample() const;
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

        void clockTimer();
        void clockEnvelope();
        void clockLength();
        float sample() const;
    };

    Pulse m_pulse1, m_pulse2;
    Triangle m_triangle;
    Noise m_noise;

    uint8_t m_frameCounter = 0;
    bool m_frameMode5 = false;  // $4017 bit 7
    bool m_irqInhibit = false;
    uint32_t m_frameCycles = 0;

    // Audio output
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

    static const uint8_t lengthTable[32];
    static const uint8_t dutyTable[4][8];
    static const uint16_t noisePeriods[16];
    static const uint8_t triangleSequence[32];
};
