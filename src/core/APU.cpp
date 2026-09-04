#include "APU.hpp"
#include "Bus.hpp"
#include "CPU.hpp"
#include "Cartridge.hpp"
#ifndef NES_HEADLESS
#include <SDL3/SDL.h>
#endif
#include <cstring>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>

namespace {
void updateOutputFilterCoefficients(int sampleRate,
                                    double& hp90, double& hp440, double& lp14k)
{
    const double rate = double(std::max(sampleRate, 1));
    constexpr double twoPi = 6.28318530717958647692;
    hp90 = std::exp(-twoPi * 90.0 / rate);
    hp440 = std::exp(-twoPi * 440.0 / rate);
    lp14k = std::exp(-twoPi * 14000.0 / rate);
}

void traceImplicitDmc(const Bus* bus, const char* event,
                      uint16_t bytesRemaining, uint16_t sampleLength,
                      bool enabled, bool loop, bool bufferFull,
                      uint8_t bitsRemaining, uint16_t timer,
                      uint8_t implicitWindow, bool abortPending,
                      bool forcedPending, bool dmaPending)
{
    (void)bus; (void)event; (void)bytesRemaining; (void)sampleLength; (void)enabled; (void)loop;
    (void)bufferFull; (void)bitsRemaining; (void)timer; (void)implicitWindow;
    (void)abortPending; (void)forcedPending; (void)dmaPending;
}
}

const uint8_t APU::lengthTable[32] = {
    10,254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
};

const uint8_t APU::dutyTable[4][8] = {
    {0,1,0,0,0,0,0,0},
    {0,1,1,0,0,0,0,0},
    {0,1,1,1,1,0,0,0},
    {1,0,0,1,1,1,1,1}
};

const uint16_t APU::noisePeriodsNtsc[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};

const uint16_t APU::noisePeriodsPal[16] = {
    4, 8, 14, 30, 60, 88, 118, 148, 188, 236, 354, 472, 708, 944, 1890, 3778
};

const uint8_t APU::triangleSequence[32] = {
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15
};

const uint16_t APU::dmcRatesNtsc[16] = {
    428, 380, 340, 320, 286, 254, 226, 214,
    190, 160, 142, 128, 106,  84,  72,  54
};

const uint16_t APU::dmcRatesPal[16] = {
    398, 354, 316, 298, 276, 236, 210, 198,
    176, 148, 132, 118,  98,  78,  66,  50
};

#ifndef NES_HEADLESS
static void SDLCALL sdlAudioCallback(void* userdata, SDL_AudioStream* stream,
                                     int additionalAmount, int )
{
    APU* apu = static_cast<APU*>(userdata);
    std::array<float, 1024> samples{};
    int bytesRemaining = additionalAmount;
    while (bytesRemaining > 0) {
        const int sampleCount = std::min<int>(
            bytesRemaining / static_cast<int>(sizeof(float)),
            static_cast<int>(samples.size()));
        if (sampleCount <= 0) break;
        apu->fillBuffer(samples.data(), sampleCount);
        const int bytes = sampleCount * static_cast<int>(sizeof(float));
        if (!SDL_PutAudioStreamData(stream, samples.data(), bytes)) break;
        bytesRemaining -= bytes;
    }
}
#endif

APU::APU()
{
    m_ring.assign(kRingSize, 0.0f);
    rebuildTriangleLoudnessTable();
    updateOutputFilterCoefficients(m_outputSampleRate, m_hp90Coefficient, m_hp440Coefficient, m_lp14kCoefficient);
    powerOn();
}
APU::~APU() { shutdownAudio(); }

void APU::setTiming(ConsoleTiming timing)
{
    if (m_timing == timing) return;
    m_timing = timing;
    m_samplePeriod = double(consoleCpuClockHz(m_timing)) / double(m_outputSampleRate);
    rebuildTriangleLoudnessTable();
    resetOutputPipeline();
}

void APU::powerOn()
{

    m_pulse1 = Pulse{};
    m_pulse2 = Pulse{};
    m_triangle = Triangle{};
    m_noise = Noise{};
    m_dmc = Dmc{};
    m_dmc.rate = (m_timing == ConsoleTiming::PAL) ? dmcRatesPal[0] : dmcRatesNtsc[0];

    m_frameMode5 = false;
    m_irqInhibit = false;
    m_frameIrqFlag = false;
    m_frameIrqClearPending = false;

    m_frameCycles = 2;
    m_apuPhase = false;
    m_frameResetDelay = 0;
    m_pendingFrameStartCycles = 0;
    m_pendingFrameMode5 = false;

    resetOutputPipeline();
}

void APU::reset()
{

    auto clearLengthChannel = [](auto& channel) {
        channel.enabled = false;
        channel.length = 0;
        channel.pendingLengthReload = false;
        channel.pendingLengthHaltValid = false;
    };

    clearLengthChannel(m_pulse1);
    clearLengthChannel(m_pulse2);
    clearLengthChannel(m_triangle);
    clearLengthChannel(m_noise);

    m_dmc.enabled = false;
    m_dmc.bytesRemaining = 0;
    m_dmc.dmaPending = false;
    m_dmc.dmaStartDelay = 0;
    m_dmc.dmaLoadPending = false;
    m_dmc.dmaReloadWaitingForEnable = false;
    m_dmc.stopBugWindow = 0;
    m_dmc.implicitStopWindow = 0;
    m_dmc.dmaAbortPending = false;
    m_dmc.dmaImplicitAbortPending = false;
    m_dmc.dmaImplicitAbortDelay = 0;
    m_dmc.dmaForcedReloadPending = false;
    m_dmc.sampleBufferFull = false;
    m_dmc.bitsRemaining = 0;
    m_dmc.silence = true;
    m_dmc.irqFlag = false;

    m_frameIrqFlag = false;
    m_frameIrqClearPending = false;
    m_frameCycles = 2;
    m_apuPhase = false;
    m_frameResetDelay = 0;
    m_pendingFrameStartCycles = 0;
    m_pendingFrameMode5 = false;

    resetOutputPipeline();
}

bool APU::initAudio()
{
#ifdef NES_HEADLESS

    m_audioOpen = false;
    return true;
#else
    if (m_audioOpen)
        return true;

    SDL_AudioSpec want{};
    want.freq = kSampleRate;
    want.format = SDL_AUDIO_F32;
    want.channels = 1;

    SDL_AudioStream* stream = SDL_OpenAudioDeviceStream(
        SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &want, sdlAudioCallback, this);
    if (!stream)
        return false;

    m_audioStream = stream;
    m_outputSampleRate = want.freq;
    m_samplePeriod = double(consoleCpuClockHz(m_timing)) / double(m_outputSampleRate);
    updateOutputFilterCoefficients(m_outputSampleRate, m_hp90Coefficient, m_hp440Coefficient, m_lp14kCoefficient);
    resetOutputPipeline();
    m_audioOpen = true;
    m_audioPlaybackPaused = true;
    return true;
#endif
}

void APU::shutdownAudio()
{
#ifdef NES_HEADLESS
    m_audioStream = nullptr;
    m_audioOpen = false;
#else
    if (!m_audioOpen)
        return;
    SDL_DestroyAudioStream(static_cast<SDL_AudioStream*>(m_audioStream));
    m_audioStream = nullptr;
    m_audioOpen = false;
#endif
}

void APU::setMasterVolume(float volume)
{
    m_masterVolume = std::clamp(volume, 0.0f, 2.5f);
}

size_t APU::queuedAudioSamples()
{
    const size_t write = m_ringWrite.load(std::memory_order_acquire);
    const size_t read = m_ringRead.load(std::memory_order_acquire);
    return (write + kRingSize - read) % kRingSize;
}

uint64_t APU::audioUnderrunCount() const
{
    return m_audioUnderruns.load(std::memory_order_relaxed);
}

uint64_t APU::audioOverrunCount() const
{
    return m_audioOverruns.load(std::memory_order_relaxed);
}

void APU::setHostAudioEnabled(bool enabled)
{
    if (m_hostAudioEnabled == enabled)
        return;
#ifndef NES_HEADLESS
    if (m_audioOpen)
        SDL_PauseAudioStreamDevice(static_cast<SDL_AudioStream*>(m_audioStream));
#endif
    m_audioPlaybackPaused = true;
    m_hostAudioEnabled = enabled;

    resetOutputPipeline();
}

void APU::setAudioPlaybackPaused(bool paused)
{
    if (!m_audioOpen || m_audioPlaybackPaused == paused)
        return;
#ifndef NES_HEADLESS
    if (paused)
        SDL_PauseAudioStreamDevice(static_cast<SDL_AudioStream*>(m_audioStream));
    else
        SDL_ResumeAudioStreamDevice(static_cast<SDL_AudioStream*>(m_audioStream));
#endif
    m_audioPlaybackPaused = paused;
}

void APU::setChipMod(bool enabled)
{
    if (m_chipMod == enabled)
        return;
    m_chipMod = enabled;

    resetOutputPipeline();
}

void APU::Pulse::clockTimer()
{
    if (timer == 0) { timer = timerPeriod; dutyBit = (dutyBit + 1) & 7; }
    else timer--;
}
void APU::Pulse::clockEnvelope()
{
    if (envelopeStart) { envelope = 15; envelopePeriod = volume; envelopeStart = false; }
    else if (envelopePeriod == 0) {
        envelopePeriod = volume;
        if (envelope > 0) envelope--;
        else if (lengthHalt) envelope = 15;
    }
    else envelopePeriod--;
}
void APU::Pulse::clockLength() { if (!lengthHalt && length > 0) length--; }
void APU::Pulse::clockSweep(bool isPulse1)
{
    if (sweepDivider == 0) {
        if (sweepEnabled && sweepShift > 0 && timerPeriod >= 8) {
            const int target = sweepTarget(isPulse1);
            if (target >= 0 && target <= 0x7FF)
                timerPeriod = static_cast<uint16_t>(target);
        }
        sweepDivider = sweepPeriod;
    }
    else sweepDivider--;
    if (sweepReload) { sweepDivider = sweepPeriod; sweepReload = false; }
}

int APU::Pulse::sweepTarget(bool isPulse1) const
{
    int delta = int(timerPeriod >> sweepShift);
    if (sweepNegate)
        delta = isPulse1 ? -delta - 1 : -delta;
    return int(timerPeriod) + delta;
}

bool APU::Pulse::muted(bool isPulse1) const
{
    if (!enabled || length == 0 || timerPeriod < 8 || timerPeriod > 0x7FF)
        return true;

    if (sweepTarget(isPulse1) > 0x7FF)
        return true;
    return false;
}

float APU::Pulse::sample(bool isPulse1) const
{
    if (muted(isPulse1) || !dutyTable[duty][dutyBit])
        return 0.0f;
    return constant ? (volume / 15.0f) : (envelope / 15.0f);
}

void APU::Triangle::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod;
        if (length > 0 && linear > 0) {
            sequencer = (sequencer + 1) & 31;

            phase = float(sequencer) / 32.0f;
        }
    }
    else timer--;
}
void APU::Triangle::clockLinear()
{
    if (linearReloadFlag) linear = linearReload;
    else if (linear > 0) linear--;
    if (!lengthHalt) linearReloadFlag = false;
}
void APU::Triangle::clockLength() { if (!lengthHalt && length > 0) length--; }
float APU::Triangle::sample(bool chipMod) const
{
    const float current = triangleSequence[sequencer] / 15.0f;

    if (!chipMod || length == 0 || linear == 0)
        return current;

    const float next = triangleSequence[(sequencer + 1) & 31] / 15.0f;
    const float denom = float(timerPeriod) + 1.0f;
    const float elapsed = float(timerPeriod - std::min<uint16_t>(timer, timerPeriod));
    const float frac = denom > 0.0f ? std::clamp(elapsed / denom, 0.0f, 1.0f) : 0.0f;
    return current + (next - current) * frac;
}

void APU::rebuildTriangleLoudnessTable()
{

    static constexpr double hz[] = {
        20,25,31.5,40,50,63,80,100,125,160,200,250,315,400,500,630,800,1000,
        1250,1600,2000,2500,3150,4000,5000,6300,8000,10000,12500,16000,20000
    };
    static constexpr double spl[] = {
        128.41,124.15,120.11,116.38,113.35,110.65,108.16,106.17,104.48,103.03,
        101.85,100.97,100.30,99.83,99.62,99.50,99.44,100.01,102.81,104.25,101.18,
        98.48,97.67,99.00,102.30,107.23,111.11,110.23,102.07,100.83,133.73
    };
    constexpr size_t count = sizeof(hz) / sizeof(hz[0]);

    for (size_t period = 0; period < m_triangleGain.size(); ++period) {
        const double frequency = double(consoleCpuClockHz(m_timing)) / (32.0 * (double(period) + 1.0));
        double contour = spl[0];
        if (frequency <= hz[0]) {
            contour = spl[0];
        }
        else if (frequency >= hz[count - 1]) {
            contour = spl[count - 1];
        }
        else {
            for (size_t i = 1; i < count; ++i) {
                if (frequency <= hz[i]) {
                    const double f = (frequency - hz[i - 1]) / (hz[i] - hz[i - 1]);
                    contour = spl[i - 1] + (spl[i] - spl[i - 1]) * f;
                    break;
                }
            }
        }
        const double loudness = (contour - 97.67) / 2.35;
        m_triangleGain[period] = float(std::pow(2.0, loudness / 6.014));
    }
}

float APU::triangleLoudnessGain(uint16_t period) const
{
    return m_triangleGain[std::min<size_t>(period, m_triangleGain.size() - 1)];
}

void APU::Noise::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod > 0 ? static_cast<uint16_t>(timerPeriod - 1) : 0;
        const uint16_t feedback = mode ? ((shift >> 6) ^ shift) & 1 : ((shift >> 1) ^ shift) & 1;
        shift = (shift >> 1) | (feedback << 14);

        smooth = (shift & 1) ? 0.0f : 1.0f;
    }
    else timer--;
}
void APU::Noise::clockEnvelope()
{
    if (envelopeStart) { envelope = 15; envelopePeriod = volume; envelopeStart = false; }
    else if (envelopePeriod == 0) {
        envelopePeriod = volume;
        if (envelope > 0) envelope--;
        else if (lengthHalt) envelope = 15;
    }
    else envelopePeriod--;
}
void APU::Noise::clockLength() { if (!lengthHalt && length > 0) length--; }
float APU::Noise::sample() const
{
    if (!enabled || length == 0 || (shift & 1)) return 0.0f;
    return constant ? (volume / 15.0f) : (envelope / 15.0f);
}

void APU::Dmc::start()
{
    currentAddr = sampleAddr;
    bytesRemaining = sampleLength;
}

void APU::Dmc::clockTimer()
{
    if (timer == 0) {

        timer = rate > 0 ? static_cast<uint16_t>(rate - 1) : 0;

        if (bitsRemaining == 0) {
            bitsRemaining = 8;
            if (sampleBufferFull) {
                silence = false;
                shiftReg = sampleBuffer;
                sampleBufferFull = false;
            }
            else {
                silence = true;
            }
        }

        if (!silence) {
            if (shiftReg & 1) {
                if (output <= 125) output = static_cast<uint8_t>(output + 2);
            }
            else {
                if (output >= 2) output = static_cast<uint8_t>(output - 2);
            }
        }

        shiftReg >>= 1;
        if (bitsRemaining > 0)
            --bitsRemaining;
    }
    else {
        --timer;
    }
}

float APU::Dmc::sample() const
{
    return output / 127.0f;
}

void APU::scheduleDmcDma()
{
    if (!m_bus || m_dmc.dmaPending)
        return;

    if (m_dmc.dmaForcedReloadPending) {
        const bool getCycle = m_bus->dmaGetCycle();
        if (!getCycle && m_bus->requestDmcDma(m_dmc.currentAddr, false)) {
            m_dmc.dmaPending = true;
            m_dmc.dmaForcedReloadPending = false;
        }
        return;
    }

    if (m_dmc.dmaAbortPending) {

        const bool getCycle = m_bus->dmaGetCycle();
        const bool mayAttemptNow = m_dmc.dmaImplicitAbortPending || !getCycle;
        if (m_dmc.sampleLength == 1)
            traceImplicitDmc(m_bus, mayAttemptNow ? "ABORT_TRY_NOW" : "ABORT_WAIT_GET", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        if (mayAttemptNow && m_bus->requestDmcDma(m_dmc.currentAddr, true)) {
            if (m_dmc.sampleLength == 1)
                traceImplicitDmc(m_bus, "ABORT_REQUESTED", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, true);
            m_dmc.dmaPending = true;
            m_dmc.dmaAbortPending = false;
            m_dmc.dmaImplicitAbortPending = false;
            m_dmc.dmaImplicitAbortDelay = 0;
        }
        return;
    }

    if (!m_dmc.enabled || m_dmc.bytesRemaining == 0)
        return;

    if (m_dmc.dmaStartDelay != 0) {
        --m_dmc.dmaStartDelay;
        if (m_dmc.dmaStartDelay != 0)
            return;
    }

    if (m_dmc.sampleBufferFull)
        return;

    const bool getCycle = m_bus->dmaGetCycle();

    const bool scheduledPhase = m_dmc.dmaReloadWaitingForEnable ? getCycle :
        (m_dmc.dmaLoadPending ? getCycle : !getCycle);
    if (!scheduledPhase)
        return;

    if (m_bus->requestDmcDma(m_dmc.currentAddr)) {
        m_dmc.dmaPending = true;
        m_dmc.dmaLoadPending = false;
        m_dmc.dmaReloadWaitingForEnable = false;
    }
}

void APU::completeDmcDma(uint8_t data)
{
    if (!m_dmc.dmaPending)
        return;

    const uint16_t fetchedAddress = m_dmc.currentAddr;
    m_dmc.dmaPending = false;
    m_dmc.sampleBuffer = data;
    m_dmc.sampleBufferFull = true;

    if (m_dmc.bytesRemaining == 0)
        return;

    ++m_dmc.currentAddr;
    if (m_dmc.currentAddr == 0)
        m_dmc.currentAddr = 0x8000;

    --m_dmc.bytesRemaining;
    if (m_dmc.bytesRemaining == 0) {
        const bool oneByteImplicitStop = !m_dmc.loop && m_dmc.sampleLength == 1;

        const bool outputReloadNextClock =
            m_dmc.bitsRemaining == 0 && m_dmc.timer == 0;
        const bool lateUnexpectedReload = oneByteImplicitStop &&
            m_dmcCpuRevision == DmcCpuRevision::Mid1990OrLater &&
            outputReloadNextClock;

        if (lateUnexpectedReload) {

            m_dmc.currentAddr = fetchedAddress;
            m_dmc.dmaForcedReloadPending = true;
            m_dmc.implicitStopWindow = 0;
            traceImplicitDmc(m_bus, "LOAD_COMPLETE_ARM_LATE_RELOAD", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        }
        else if (oneByteImplicitStop && !outputReloadNextClock) {

            m_dmc.implicitStopWindow = 3;
            traceImplicitDmc(m_bus, "LOAD_COMPLETE_ARM_WINDOW", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        }

        if (m_dmc.loop) {
            m_dmc.start();
        }
        else {
            if (m_dmc.irqEnabled)
                m_dmc.irqFlag = true;

        }
    }
}

void APU::abortDmcDma()
{

    m_dmc.dmaPending = false;
}

void APU::quarterFrame()
{
    m_pulse1.clockEnvelope();
    m_pulse2.clockEnvelope();
    m_noise.clockEnvelope();
    m_triangle.clockLinear();
}
void APU::halfFrame()
{
    auto clockLength = [](auto& channel) {

        if (channel.pendingLengthReload) {
            if (channel.length > 0)
                channel.pendingLengthReload = false;
            else
                return;
        }

        channel.clockLength();
    };

    clockLength(m_pulse1);
    clockLength(m_pulse2);
    clockLength(m_noise);
    clockLength(m_triangle);
    m_pulse1.clockSweep(true);
    m_pulse2.clockSweep(false);
}

void APU::applyPendingLengthWrites()
{
    auto apply = [](auto& channel) {

        if (channel.pendingLengthHaltValid) {
            channel.lengthHalt = channel.pendingLengthHalt;
            channel.pendingLengthHaltValid = false;
        }

        if (channel.pendingLengthReload) {
            if (channel.enabled)
                channel.length = channel.pendingLengthValue;
            channel.pendingLengthReload = false;
        }
    };

    apply(m_pulse1);
    apply(m_pulse2);
    apply(m_triangle);
    apply(m_noise);
}

void APU::clockFrameCounterPreCpuPhase()
{

    if (m_frameResetDelay != 1)
        return;

    m_frameResetDelay = 0;
    m_frameMode5 = m_pendingFrameMode5;
    m_frameCycles = m_pendingFrameStartCycles;
    if (m_frameMode5) {
        quarterFrame();
        halfFrame();
    }
    m_frameResetAppliedPreCpu = true;
}

void APU::clockFrameCounter()
{
    const bool resetAppliedPreCpu = m_frameResetAppliedPreCpu;
    m_frameResetAppliedPreCpu = false;

    if (resetAppliedPreCpu && m_frameResetDelay == 0) {
        applyPendingLengthWrites();
        return;
    }

    const bool enteringGetPhase = m_bus ? !m_bus->dmaGetCycle() : !m_apuPhase;
    if (m_frameIrqClearPending && enteringGetPhase) {
        m_frameIrqFlag = false;
        m_frameIrqClearPending = false;
    }

    if (m_frameResetDelay != 0) {
        --m_frameResetDelay;
        if (m_frameResetDelay == 0) {
            m_frameMode5 = m_pendingFrameMode5;

            m_frameCycles = m_pendingFrameStartCycles;

            if (m_frameMode5) {
                quarterFrame();
                halfFrame();
            }
        }

        applyPendingLengthWrites();
        return;
    }

    ++m_frameCycles;

    if (!m_frameMode5) {
        const uint32_t q1 = (m_timing == ConsoleTiming::PAL) ? 8313u : 7457u;
        const uint32_t qh2 = (m_timing == ConsoleTiming::PAL) ? 16627u : 14913u;
        const uint32_t q3 = (m_timing == ConsoleTiming::PAL) ? 24939u : 22371u;
        const uint32_t irq0 = (m_timing == ConsoleTiming::PAL) ? 33252u : 29828u;
        const uint32_t qh4 = (m_timing == ConsoleTiming::PAL) ? 33253u : 29829u;
        const uint32_t reset = (m_timing == ConsoleTiming::PAL) ? 33254u : 29830u;

        if (m_frameCycles == q1) {
            quarterFrame();
        }
        else if (m_frameCycles == qh2) {
            quarterFrame();
            halfFrame();
        }
        else if (m_frameCycles == q3) {
            quarterFrame();
        }
        else if (m_frameCycles == irq0) {

            m_frameIrqFlag = true;
        }
        else if (m_frameCycles == qh4) {
            quarterFrame();
            halfFrame();
            m_frameIrqFlag = true;
        }
        else if (m_frameCycles == reset) {

            m_frameIrqFlag = !m_irqInhibit;
            m_frameCycles = 0;
        }
    }
    else {
        const uint32_t q1 = (m_timing == ConsoleTiming::PAL) ? 8313u : 7457u;
        const uint32_t qh2 = (m_timing == ConsoleTiming::PAL) ? 16627u : 14913u;
        const uint32_t q3 = (m_timing == ConsoleTiming::PAL) ? 24939u : 22371u;

        const uint32_t qh5 = (m_timing == ConsoleTiming::PAL) ? 41565u : 37281u;

        if (m_frameCycles == q1) {
            quarterFrame();
        }
        else if (m_frameCycles == qh2) {
            quarterFrame();
            halfFrame();
        }
        else if (m_frameCycles == q3) {
            quarterFrame();
        }
        else if (m_frameCycles == qh5) {
            quarterFrame();
            halfFrame();

            m_frameCycles = UINT32_MAX;
        }
    }

    applyPendingLengthWrites();
}

void APU::clockFrameCounterPhase()
{
    clockFrameCounter();
}

void APU::pushSample(float s)
{
    if (!m_hostAudioEnabled)
        return;

    const size_t write = m_ringWrite.load(std::memory_order_relaxed);
    const size_t next = (write + 1) % kRingSize;
    const size_t read = m_ringRead.load(std::memory_order_acquire);
    if (next == read) {

        m_audioOverruns.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    m_ring[write] = s;
    m_ringWrite.store(next, std::memory_order_release);
}

float APU::mixInstantSample() const
{
    const float p1 = m_pulse1.sample(true);
    const float p2 = m_pulse2.sample(false);
    float t = m_triangle.sample(m_chipMod);
    const float n = m_noise.sample();
    const float d = m_dmc.sample();

    float base = 0.0f;
    if (m_chipMod) {

        constexpr float pulseGain = 95.88f / ((8128.0f / 15.0f) + 100.0f);
        constexpr float triangleGain = 0.95f * 45.0f / 208.0f;
        constexpr float noiseGain = 0.95f * 15.0f * 1.95f / 208.0f;
        constexpr float dmcGain = 0.95f * 127.0f / 208.0f;

        const float gain = (m_triangle.timerPeriod > 2)
            ? triangleLoudnessGain(m_triangle.timerPeriod)
            : 1.0f;
        t = (t - 0.5f) * gain + 0.5f;
        base = p1 * pulseGain + p2 * pulseGain + t * triangleGain +
               n * noiseGain + d * dmcGain;
    }
    else {
        float pulseOut = 0.0f;
        if (p1 + p2 > 0.0f)
            pulseOut = 95.88f / ((8128.0f / (p1 * 15.0f + p2 * 15.0f)) + 100.0f);

        float tnd = 0.0f;
        const float tndSum = t * 15.0f / 8227.0f + n * 15.0f / 12241.0f + d * 127.0f / 22638.0f;
        if (tndSum > 0.0f)
            tnd = 159.79f / (1.0f / tndSum + 100.0f);
        base = pulseOut + tnd;
    }

    const float expansion = m_cart ? m_cart->expansionAudioSample(m_chipMod) : 0.0f;
    return base + expansion;
}

float APU::filterOutput(float s)
{

    const double x0 = s;
    const double hp90 = x0 - m_hp90PrevInput + m_hp90Coefficient * m_hp90PrevOutput;
    m_hp90PrevInput = x0;
    m_hp90PrevOutput = hp90;

    const double hp440 = hp90 - m_hp440PrevInput + m_hp440Coefficient * m_hp440PrevOutput;
    m_hp440PrevInput = hp90;
    m_hp440PrevOutput = hp440;

    const double lp14k = (1.0 - m_lp14kCoefficient) * hp440 +
                         m_lp14kCoefficient * m_lp14kPrevOutput;
    m_lp14kPrevOutput = lp14k;

    constexpr double outputHeadroom = 0.85;
    return std::clamp(float(lp14k * outputHeadroom * double(m_masterVolume)), -1.0f, 1.0f);
}

void APU::resetOutputPipeline()
{
    m_sampleTimer = 0.0;
    m_mixAccumulator = 0.0;
    m_mixAccumulatorWeight = 0.0;
    m_hp90PrevInput = 0.0;
    m_hp90PrevOutput = 0.0;
    m_hp440PrevInput = 0.0;
    m_hp440PrevOutput = 0.0;
    m_lp14kPrevOutput = 0.0;

    m_ringRead.store(0, std::memory_order_release);
    m_ringWrite.store(0, std::memory_order_release);
    m_audioUnderruns.store(0, std::memory_order_relaxed);
    m_audioOverruns.store(0, std::memory_order_relaxed);
}

void APU::clock()
{

    m_apuPhase = !m_apuPhase;
    if (m_apuPhase) {
        m_pulse1.clockTimer();
        m_pulse2.clockTimer();
    }

    m_triangle.clockTimer();
    m_noise.clockTimer();

    const bool dmcBufferWasFull = m_dmc.sampleBufferFull;
    m_dmc.clockTimer();
    const bool dmcBufferEmptied = dmcBufferWasFull && !m_dmc.sampleBufferFull;

    if (dmcBufferEmptied) {
        if (m_dmc.enabled && m_dmc.bytesRemaining != 0 &&
            !m_dmc.dmaLoadPending && m_dmc.dmaStartDelay != 0) {
            m_dmc.dmaReloadWaitingForEnable = true;
        }
        if (m_dmc.sampleLength == 1)
            traceImplicitDmc(m_bus, "BUFFER_EMPTIED", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);

        if (m_dmc.stopBugWindow == 3)
            m_dmc.dmaForcedReloadPending = true;
        else if (m_dmc.stopBugWindow != 0 || m_dmc.implicitStopWindow != 0) {

            const bool implicitAbort = (m_dmc.stopBugWindow == 0 && m_dmc.implicitStopWindow != 0);
            m_dmc.dmaAbortPending = true;
            m_dmc.dmaImplicitAbortPending = implicitAbort;

            m_dmc.dmaImplicitAbortDelay = 0;

            if (m_dmc.sampleLength == 1)
                traceImplicitDmc(m_bus, "BUFFER_EMPTY_ARM_ABORT", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        }
        m_dmc.stopBugWindow = 0;
        m_dmc.implicitStopWindow = 0;
    }
    else {
        if (m_dmc.stopBugWindow != 0)
            --m_dmc.stopBugWindow;
        if (m_dmc.implicitStopWindow != 0)
            --m_dmc.implicitStopWindow;
    }

    scheduleDmcDma();

    const double instant = double(mixInstantSample());
    double remaining = 1.0;
    constexpr double epsilon = 1.0e-12;
    while (remaining > epsilon) {
        const double room = std::max(0.0, m_samplePeriod - m_sampleTimer);
        const double weight = std::min(remaining, room);
        if (weight <= epsilon) {

            m_sampleTimer = m_samplePeriod;
        }
        else {
            m_mixAccumulator += instant * weight;
            m_mixAccumulatorWeight += weight;
            m_sampleTimer += weight;
            remaining -= weight;
        }

        if (m_sampleTimer + epsilon >= m_samplePeriod) {
            const float averaged = m_mixAccumulatorWeight > 0.0
                ? float(m_mixAccumulator / m_mixAccumulatorWeight)
                : 0.0f;
            pushSample(filterOutput(averaged));
            m_sampleTimer = std::max(0.0, m_sampleTimer - m_samplePeriod);
            m_mixAccumulator = 0.0;
            m_mixAccumulatorWeight = 0.0;
        }
    }
}

void APU::fillBuffer(float* stream, int len)
{
    size_t read = m_ringRead.load(std::memory_order_relaxed);
    const size_t write = m_ringWrite.load(std::memory_order_acquire);
    bool underrun = false;

    for (int i = 0; i < len; ++i) {
        if (read != write) {
            stream[i] = m_ring[read];
            read = (read + 1) % kRingSize;
        }
        else {
            stream[i] = 0.0f;
            underrun = true;
        }
    }
    m_ringRead.store(read, std::memory_order_release);
    if (underrun)
        m_audioUnderruns.fetch_add(1, std::memory_order_relaxed);
}

uint8_t APU::cpuRead(uint16_t addr) const
{
    if (addr == 0x4015) {
        uint8_t v = 0;
        if (m_pulse1.length > 0) v |= 0x01;
        if (m_pulse2.length > 0) v |= 0x02;
        if (m_triangle.length > 0) v |= 0x04;
        if (m_noise.length > 0) v |= 0x08;
        if (m_dmc.bytesRemaining > 0) v |= 0x10;
        if (m_frameIrqFlag) v |= 0x40;
        if (m_dmc.irqFlag) v |= 0x80;

        m_frameIrqClearPending = true;
        return v;
    }
    return 0;
}

void APU::cpuWrite(uint16_t addr, uint8_t data)
{
    switch (addr) {
    case 0x4000:
        m_pulse1.duty = (data >> 6) & 3;
        m_pulse1.pendingLengthHalt = (data & 0x20) != 0;
        m_pulse1.pendingLengthHaltValid = true;
        m_pulse1.constant = (data & 0x10) != 0;
        m_pulse1.volume = data & 0x0F;
        break;
    case 0x4001:
        m_pulse1.sweepEnabled = (data & 0x80) != 0;
        m_pulse1.sweepPeriod = (data >> 4) & 7;
        m_pulse1.sweepNegate = (data & 0x08) != 0;
        m_pulse1.sweepShift = data & 7;
        m_pulse1.sweepReload = true;
        break;
    case 0x4002:
        m_pulse1.timerPeriod = (m_pulse1.timerPeriod & 0xFF00) | data;
        break;
    case 0x4003:
        m_pulse1.timerPeriod = (m_pulse1.timerPeriod & 0x00FF) | ((data & 7) << 8);
        m_pulse1.pendingLengthValue = lengthTable[(data >> 3) & 0x1F];
        m_pulse1.pendingLengthReload = true;
        m_pulse1.dutyBit = 0;
        m_pulse1.envelopeStart = true;
        break;

    case 0x4004:
        m_pulse2.duty = (data >> 6) & 3;
        m_pulse2.pendingLengthHalt = (data & 0x20) != 0;
        m_pulse2.pendingLengthHaltValid = true;
        m_pulse2.constant = (data & 0x10) != 0;
        m_pulse2.volume = data & 0x0F;
        break;
    case 0x4005:
        m_pulse2.sweepEnabled = (data & 0x80) != 0;
        m_pulse2.sweepPeriod = (data >> 4) & 7;
        m_pulse2.sweepNegate = (data & 0x08) != 0;
        m_pulse2.sweepShift = data & 7;
        m_pulse2.sweepReload = true;
        break;
    case 0x4006:
        m_pulse2.timerPeriod = (m_pulse2.timerPeriod & 0xFF00) | data;
        break;
    case 0x4007:
        m_pulse2.timerPeriod = (m_pulse2.timerPeriod & 0x00FF) | ((data & 7) << 8);
        m_pulse2.pendingLengthValue = lengthTable[(data >> 3) & 0x1F];
        m_pulse2.pendingLengthReload = true;
        m_pulse2.dutyBit = 0;
        m_pulse2.envelopeStart = true;
        break;

    case 0x4008:
        m_triangle.pendingLengthHalt = (data & 0x80) != 0;
        m_triangle.pendingLengthHaltValid = true;
        m_triangle.linearReload = data & 0x7F;
        break;
    case 0x400A:
        m_triangle.timerPeriod = (m_triangle.timerPeriod & 0xFF00) | data;
        break;
    case 0x400B:
        m_triangle.timerPeriod = (m_triangle.timerPeriod & 0x00FF) | ((data & 7) << 8);
        m_triangle.pendingLengthValue = lengthTable[(data >> 3) & 0x1F];
        m_triangle.pendingLengthReload = true;
        m_triangle.linearReloadFlag = true;
        break;

    case 0x400C:
        m_noise.pendingLengthHalt = (data & 0x20) != 0;
        m_noise.pendingLengthHaltValid = true;
        m_noise.constant = (data & 0x10) != 0;
        m_noise.volume = data & 0x0F;
        break;
    case 0x400E:
        m_noise.mode = (data & 0x80) != 0;
        m_noise.timerPeriod = (m_timing == ConsoleTiming::PAL ? noisePeriodsPal : noisePeriodsNtsc)[data & 0x0F];
        break;
    case 0x400F:
        m_noise.pendingLengthValue = lengthTable[(data >> 3) & 0x1F];
        m_noise.pendingLengthReload = true;
        m_noise.envelopeStart = true;
        break;

    case 0x4010:
        m_dmc.irqEnabled = (data & 0x80) != 0;
        m_dmc.loop = (data & 0x40) != 0;
        m_dmc.rate = (m_timing == ConsoleTiming::PAL ? dmcRatesPal : dmcRatesNtsc)[data & 0x0F];
        if (!m_dmc.irqEnabled)
            m_dmc.irqFlag = false;
        break;
    case 0x4011:
        m_dmc.output = data & 0x7F;
        break;
    case 0x4012:
        m_dmc.sampleAddr = 0xC000 | ((uint16_t)data << 6);
        break;
    case 0x4013:
        m_dmc.sampleLength = ((uint16_t)data << 4) + 1;
        break;

    case 0x4015: {
        m_pulse1.enabled = (data & 0x01) != 0;
        m_pulse2.enabled = (data & 0x02) != 0;
        m_triangle.enabled = (data & 0x04) != 0;
        m_noise.enabled = (data & 0x08) != 0;
        const bool dmcEnableWrite = (data & 0x10) != 0;
        if (m_dmc.sampleLength == 1 || dmcEnableWrite)
            traceImplicitDmc(m_bus, dmcEnableWrite ? "WRITE4015_ENABLE" : "WRITE4015_DISABLE", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        if (!m_pulse1.enabled) { m_pulse1.length = 0; m_pulse1.pendingLengthReload = false; }
        if (!m_pulse2.enabled) { m_pulse2.length = 0; m_pulse2.pendingLengthReload = false; }
        if (!m_triangle.enabled) { m_triangle.length = 0; m_triangle.pendingLengthReload = false; }
        if (!m_noise.enabled) { m_noise.length = 0; m_noise.pendingLengthReload = false; }
        if (!dmcEnableWrite) {

            m_dmc.enabled = false;
            m_dmc.bytesRemaining = 0;
            m_dmc.dmaStartDelay = 0;
            m_dmc.dmaLoadPending = false;
            m_dmc.dmaReloadWaitingForEnable = false;
            m_dmc.stopBugWindow = 3;
        }
        else {
            m_dmc.stopBugWindow = 0;
            m_dmc.implicitStopWindow = 0;
            m_dmc.enabled = true;
            if (m_dmc.bytesRemaining == 0) {
                m_dmc.dmaAbortPending = false;
                m_dmc.dmaImplicitAbortPending = false;
                m_dmc.dmaImplicitAbortDelay = 0;
                m_dmc.dmaForcedReloadPending = false;
                m_dmc.dmaReloadWaitingForEnable = false;
                m_dmc.start();
                m_dmc.dmaStartDelay = 3;

                m_dmc.dmaLoadPending = !m_dmc.sampleBufferFull;
            }
        }

        m_dmc.irqFlag = false;
        break;
    }

    case 0x4017:

        if (m_frameResetDelay == 1) {
            m_frameResetDelay = 0;
            m_frameMode5 = m_pendingFrameMode5;
            m_frameCycles = m_pendingFrameStartCycles;
            if (m_frameMode5) { quarterFrame(); halfFrame(); }
        }

        m_irqInhibit = (data & 0x40) != 0;
        if (m_irqInhibit)
            m_frameIrqFlag = false;

        m_pendingFrameMode5 = (data & 0x80) != 0;

        if (m_apuPhase) {
            m_frameResetDelay = 4;
            m_pendingFrameStartCycles = 1;
        }
        else {
            m_frameResetDelay = 5;

            m_pendingFrameStartCycles = 1;
        }
        break;
    }
}

void APU::saveState(std::vector<uint8_t>& out) const
{
    auto put8 = [&](uint8_t v) { out.push_back(v); };
    auto put16 = [&](uint16_t v) {
        out.push_back(v & 0xFF);
        out.push_back((v >> 8) & 0xFF);
    };
    auto put32 = [&](uint32_t v) {
        for (int i = 0; i < 4; ++i)
            out.push_back((v >> (i * 8)) & 0xFF);
    };
    auto put64 = [&](uint64_t v) {
        for (int i = 0; i < 8; ++i)
            out.push_back((v >> (i * 8)) & 0xFF);
    };
    auto putFloat = [&](float v) {
        uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(v), "unexpected float size");
        std::memcpy(&bits, &v, sizeof(bits));
        put32(bits);
    };
    auto putDouble = [&](double v) {
        uint64_t bits = 0;
        static_assert(sizeof(bits) == sizeof(v), "unexpected double size");
        std::memcpy(&bits, &v, sizeof(bits));
        put64(bits);
    };

    auto putPulse = [&](const Pulse& c) {
        put8(c.enabled ? 1 : 0);
        put8(c.duty);
        put8(c.volume);
        put8(c.constant ? 1 : 0);
        put8(c.lengthHalt ? 1 : 0);
        put8(c.pendingLengthHaltValid ? 1 : 0);
        put8(c.pendingLengthHalt ? 1 : 0);
        put8(c.pendingLengthReload ? 1 : 0);
        put8(c.pendingLengthValue);
        put16(c.timer);
        put16(c.timerPeriod);
        put8(c.length);
        put8(c.dutyBit);
        put8(c.envelope);
        put8(c.envelopePeriod);
        put8(c.envelopeStart ? 1 : 0);
        put8(c.sweepShift);
        put8(c.sweepNegate ? 1 : 0);
        put8(c.sweepPeriod);
        put8(c.sweepEnabled ? 1 : 0);
        put8(c.sweepReload ? 1 : 0);
        put8(c.sweepDivider);
    };

    putPulse(m_pulse1);
    putPulse(m_pulse2);

    put8(m_triangle.enabled ? 1 : 0);
    put8(m_triangle.lengthHalt ? 1 : 0);
    put8(m_triangle.pendingLengthHaltValid ? 1 : 0);
    put8(m_triangle.pendingLengthHalt ? 1 : 0);
    put8(m_triangle.pendingLengthReload ? 1 : 0);
    put8(m_triangle.pendingLengthValue);
    put16(m_triangle.timer);
    put16(m_triangle.timerPeriod);
    put8(m_triangle.length);
    put8(m_triangle.linear);
    put8(m_triangle.linearReload);
    put8(m_triangle.linearReloadFlag ? 1 : 0);
    put8(m_triangle.sequencer);
    putFloat(m_triangle.phase);

    put8(m_noise.enabled ? 1 : 0);
    put8(m_noise.lengthHalt ? 1 : 0);
    put8(m_noise.pendingLengthHaltValid ? 1 : 0);
    put8(m_noise.pendingLengthHalt ? 1 : 0);
    put8(m_noise.pendingLengthReload ? 1 : 0);
    put8(m_noise.pendingLengthValue);
    put8(m_noise.constant ? 1 : 0);
    put8(m_noise.volume);
    put8(m_noise.envelope);
    put8(m_noise.envelopePeriod);
    put8(m_noise.envelopeStart ? 1 : 0);
    put16(m_noise.timer);
    put16(m_noise.timerPeriod);
    put8(m_noise.length);
    put16(m_noise.shift);
    put8(m_noise.mode ? 1 : 0);
    putFloat(m_noise.smooth);

    put8(m_dmc.enabled ? 1 : 0);
    put8(m_dmc.irqEnabled ? 1 : 0);
    put8(m_dmc.loop ? 1 : 0);
    put8(m_dmc.irqFlag ? 1 : 0);
    put16(m_dmc.rate);
    put16(m_dmc.timer);
    put8(m_dmc.output);
    put8(m_dmc.sampleBuffer);
    put8(m_dmc.sampleBufferFull ? 1 : 0);
    put8(m_dmc.shiftReg);
    put8(m_dmc.bitsRemaining);
    put8(m_dmc.silence ? 1 : 0);
    put16(m_dmc.sampleAddr);
    put16(m_dmc.sampleLength);
    put16(m_dmc.currentAddr);
    put16(m_dmc.bytesRemaining);
    put8(m_dmc.dmaPending ? 1 : 0);
    put8(m_dmc.dmaStartDelay);
    put8(m_dmc.dmaLoadPending ? 1 : 0);
    put8(m_dmc.dmaReloadWaitingForEnable ? 1 : 0);
    put8(m_dmc.stopBugWindow);
    put8(m_dmc.implicitStopWindow);
    put8(m_dmc.dmaAbortPending ? 1 : 0);
    put8(m_dmc.dmaImplicitAbortPending ? 1 : 0);
    put8(m_dmc.dmaImplicitAbortDelay);
    put8(m_dmc.dmaForcedReloadPending ? 1 : 0);
    put8(static_cast<uint8_t>(m_dmcCpuRevision));

    put8(m_frameMode5 ? 1 : 0);
    put8(m_frameResetAppliedPreCpu ? 1 : 0);
    put8(m_irqInhibit ? 1 : 0);
    put8(m_frameIrqFlag ? 1 : 0);
    put8(m_frameIrqClearPending ? 1 : 0);
    put32(m_frameCycles);
    put8(m_apuPhase ? 1 : 0);
    put8(m_frameResetDelay);
    put8(m_pendingFrameStartCycles);
    put8(m_pendingFrameMode5 ? 1 : 0);
    put8(m_chipMod ? 1 : 0);
    putDouble(m_sampleTimer);
}

bool APU::loadState(const uint8_t*& p, const uint8_t* end)
{
    auto need = [&](size_t n) { return p + n <= end; };
    auto get8 = [&](uint8_t& v) -> bool {
        if (!need(1)) return false;
        v = *p++;
        return true;
    };
    auto get16 = [&](uint16_t& v) -> bool {
        if (!need(2)) return false;
        v = p[0] | (uint16_t(p[1]) << 8);
        p += 2;
        return true;
    };
    auto get32 = [&](uint32_t& v) -> bool {
        if (!need(4)) return false;
        v = p[0] | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
        p += 4;
        return true;
    };
    auto get64 = [&](uint64_t& v) -> bool {
        if (!need(8)) return false;
        v = 0;
        for (int i = 0; i < 8; ++i)
            v |= uint64_t(*p++) << (i * 8);
        return true;
    };
    auto getFloat = [&](float& v) -> bool {
        uint32_t bits = 0;
        if (!get32(bits)) return false;
        std::memcpy(&v, &bits, sizeof(v));
        return true;
    };
    auto getDouble = [&](double& v) -> bool {
        uint64_t bits = 0;
        if (!get64(bits)) return false;
        std::memcpy(&v, &bits, sizeof(v));
        return true;
    };
    auto getBool = [&](bool& v) -> bool {
        uint8_t b = 0;
        if (!get8(b)) return false;
        v = b != 0;
        return true;
    };

    auto getPulse = [&](Pulse& c) -> bool {
        return getBool(c.enabled) &&
            get8(c.duty) &&
            get8(c.volume) &&
            getBool(c.constant) &&
            getBool(c.lengthHalt) &&
            getBool(c.pendingLengthHaltValid) &&
            getBool(c.pendingLengthHalt) &&
            getBool(c.pendingLengthReload) &&
            get8(c.pendingLengthValue) &&
            get16(c.timer) &&
            get16(c.timerPeriod) &&
            get8(c.length) &&
            get8(c.dutyBit) &&
            get8(c.envelope) &&
            get8(c.envelopePeriod) &&
            getBool(c.envelopeStart) &&
            get8(c.sweepShift) &&
            getBool(c.sweepNegate) &&
            get8(c.sweepPeriod) &&
            getBool(c.sweepEnabled) &&
            getBool(c.sweepReload) &&
            get8(c.sweepDivider);
    };

    if (!getPulse(m_pulse1) || !getPulse(m_pulse2)) return false;

    if (!getBool(m_triangle.enabled) || !getBool(m_triangle.lengthHalt) ||
        !getBool(m_triangle.pendingLengthHaltValid) || !getBool(m_triangle.pendingLengthHalt) ||
        !getBool(m_triangle.pendingLengthReload) || !get8(m_triangle.pendingLengthValue) ||
        !get16(m_triangle.timer) || !get16(m_triangle.timerPeriod) ||
        !get8(m_triangle.length) || !get8(m_triangle.linear) ||
        !get8(m_triangle.linearReload) || !getBool(m_triangle.linearReloadFlag) ||
        !get8(m_triangle.sequencer) || !getFloat(m_triangle.phase)) return false;

    if (!getBool(m_noise.enabled) || !getBool(m_noise.lengthHalt) ||
        !getBool(m_noise.pendingLengthHaltValid) || !getBool(m_noise.pendingLengthHalt) ||
        !getBool(m_noise.pendingLengthReload) || !get8(m_noise.pendingLengthValue) ||
        !getBool(m_noise.constant) || !get8(m_noise.volume) ||
        !get8(m_noise.envelope) || !get8(m_noise.envelopePeriod) ||
        !getBool(m_noise.envelopeStart) || !get16(m_noise.timer) ||
        !get16(m_noise.timerPeriod) || !get8(m_noise.length) ||
        !get16(m_noise.shift) || !getBool(m_noise.mode) ||
        !getFloat(m_noise.smooth)) return false;

    if (!getBool(m_dmc.enabled) || !getBool(m_dmc.irqEnabled) ||
        !getBool(m_dmc.loop) || !getBool(m_dmc.irqFlag) ||
        !get16(m_dmc.rate) || !get16(m_dmc.timer) ||
        !get8(m_dmc.output) || !get8(m_dmc.sampleBuffer) ||
        !getBool(m_dmc.sampleBufferFull) || !get8(m_dmc.shiftReg) ||
        !get8(m_dmc.bitsRemaining) || !getBool(m_dmc.silence) ||
        !get16(m_dmc.sampleAddr) || !get16(m_dmc.sampleLength) ||
        !get16(m_dmc.currentAddr) || !get16(m_dmc.bytesRemaining) ||
        !getBool(m_dmc.dmaPending) || !get8(m_dmc.dmaStartDelay) ||
        !getBool(m_dmc.dmaLoadPending) || !getBool(m_dmc.dmaReloadWaitingForEnable) ||
        !get8(m_dmc.stopBugWindow) ||
        !get8(m_dmc.implicitStopWindow) || !getBool(m_dmc.dmaAbortPending) ||
        !getBool(m_dmc.dmaImplicitAbortPending) || !get8(m_dmc.dmaImplicitAbortDelay) ||
        !getBool(m_dmc.dmaForcedReloadPending)) return false;
    uint8_t dmcRevision = 0;
    if (!get8(dmcRevision) || dmcRevision > static_cast<uint8_t>(DmcCpuRevision::Mid1990OrLater))
        return false;
    m_dmcCpuRevision = static_cast<DmcCpuRevision>(dmcRevision);
    if (m_dmc.dmaStartDelay > 3 || m_dmc.stopBugWindow > 3 || m_dmc.implicitStopWindow > 3 || m_dmc.dmaImplicitAbortDelay > 1) return false;

    if (!getBool(m_frameMode5) || !getBool(m_frameResetAppliedPreCpu) || !getBool(m_irqInhibit) ||
        !getBool(m_frameIrqFlag) || !getBool(m_frameIrqClearPending) || !get32(m_frameCycles) ||
        !getBool(m_apuPhase) || !get8(m_frameResetDelay) ||
        !get8(m_pendingFrameStartCycles) || !getBool(m_pendingFrameMode5) || !getBool(m_chipMod) ||
        !getDouble(m_sampleTimer)) return false;
    if (m_frameResetDelay > 5 || m_pendingFrameStartCycles > 2) return false;

    const double restoredSampleTimer = m_sampleTimer;
    resetOutputPipeline();
    m_sampleTimer = restoredSampleTimer;

    return true;
}
