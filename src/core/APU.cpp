#include "APU.hpp"
#include "Bus.hpp"
#include "CPU.hpp"
#include "Cartridge.hpp"
#ifndef NES_HEADLESS
#include <SDL.h>
#endif
#include <cstring>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>


namespace {
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
static void sdlAudioCallback(void* userdata, Uint8* stream, int len)
{
    APU* apu = static_cast<APU*>(userdata);
    float* out = reinterpret_cast<float*>(stream);
    apu->fillBuffer(out, len / (int)sizeof(float));
}
#endif

APU::APU()
{
    m_ring.assign(kRingSize, 0.0f);
    rebuildTriangleLoudnessTable();
    m_hpCoefficient = std::exp(-2.0 * 3.14159265358979323846 * 20.0 / double(m_outputSampleRate));
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
    // Power-on initializes the APU register/control state. The reset tests
    // observe this as an effective $4015=$00 and $4017=$00 before the CPU
    // begins executing from the reset vector.
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

    // CPU::powerOn() contributes seven reset clocks. Starting the frame
    // sequencer at two makes the effective $4017=$00 write nine clocks before
    // the first instruction, inside the 9-12 clock hardware window.
    m_frameCycles = 2;
    m_apuPhase = false;
    m_frameResetDelay = 0;
    m_pendingFrameStartCycles = 0;
    m_pendingFrameMode5 = false;

    resetOutputPipeline();
}

void APU::reset()
{
    // RESET is not an APU power cycle. Hardware behaves as though $4015 is
    // cleared, but the channel register/control latches survive. In
    // particular, the triangle control/length-halt bit must survive reset.
    // Clearing the channel-enable latch immediately clears each length
    // counter, just like a CPU write of $00 to $4015.
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

    // $4015 also stops DMC playback and clears the DMC IRQ. Preserve the DMC
    // programming registers/DAC value, but cancel any in-flight sample/DMA.
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

    // The frame IRQ flag is cleared by reset. The frame-counter *mode* is not
    // reset to zero: hardware effectively rewrites the last value written to
    // $4017. Preserve mode/inhibit and restart that sequence at the same
    // reset-relative phase used at power-on.
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
    // Regression runners clock the APU exactly like the normal emulator but
    // never open a host audio device. Keeping host I/O out of the test target
    // makes timing tests deterministic and removes the SDL dependency.
    m_audioOpen = false;
    return true;
#else
    if (m_audioOpen)
        return true;

    SDL_AudioSpec want{}, have{};
    want.freq = kSampleRate;
    want.format = AUDIO_F32SYS;
    want.channels = 1;
    want.samples = 512;
    want.callback = sdlAudioCallback;
    want.userdata = this;

    // Keep the callback contract fixed at float32 mono and let SDL convert
    // to the physical device's channel count/PCM format. Only sample rate is
    // allowed to change because the resampler below follows the obtained rate.
    // The old legacy SDL_OpenAudio(..., &have) path allowed *any* change, then
    // rejected common stereo devices and left the emulator completely silent.
    const SDL_AudioDeviceID device = SDL_OpenAudioDevice(
        nullptr, 0, &want, &have, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
    if (device == 0)
        return false;

    // Format and channels were not allowed to change, so SDL should preserve
    // the callback layout and convert internally if hardware needs otherwise.
    if (have.format != AUDIO_F32SYS || have.channels != 1 || have.freq <= 0) {
        SDL_CloseAudioDevice(device);
        return false;
    }

    m_audioDeviceId = uint32_t(device);
    m_outputSampleRate = have.freq;
    m_samplePeriod = double(consoleCpuClockHz(m_timing)) / double(m_outputSampleRate);
    m_hpCoefficient = std::exp(-2.0 * 3.14159265358979323846 * 20.0 / double(m_outputSampleRate));
    resetOutputPipeline();
    m_audioOpen = true;
    SDL_PauseAudioDevice(device, 0);
    return true;
#endif
}

void APU::shutdownAudio()
{
#ifdef NES_HEADLESS
    m_audioDeviceId = 0;
    m_audioOpen = false;
#else
    if (!m_audioOpen)
        return;
    SDL_CloseAudioDevice(SDL_AudioDeviceID(m_audioDeviceId));
    m_audioDeviceId = 0;
    m_audioOpen = false;
#endif
}

void APU::setMasterVolume(float volume)
{
    m_masterVolume = std::clamp(volume, 0.0f, 2.5f);
}

void APU::setChipMod(bool enabled)
{
    if (m_chipMod == enabled)
        return;
    m_chipMod = enabled;

    // Waveform state remains untouched so the mode switch does not restart
    // music. Only discard queued/output-filter history from the old mixer.
    resetOutputPipeline();
}

// ---- Pulse ----
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

    // The sweep unit's target calculation mutes the pulse immediately when a
    // positive target would exceed 11 bits; it does not wait for the next
    // half-frame sweep clock to attempt the write.
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

// ---- Triangle ----
void APU::Triangle::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod;
        if (length > 0 && linear > 0) {
            sequencer = (sequencer + 1) & 31;
            // Kept in the legacy save-state payload for compatibility.
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

    // The triangle DAC is not gated to zero when the length/linear counter
    // stops it; the sequencer simply freezes at its current DAC level.
    if (!chipMod || length == 0 || linear == 0)
        return current;

    // KYLXBN's smooth-triangle idea is represented as a continuous ramp
    // between adjacent hardware DAC steps, phase-locked to the real timer.
    const float next = triangleSequence[(sequencer + 1) & 31] / 15.0f;
    const float denom = float(timerPeriod) + 1.0f;
    const float elapsed = float(timerPeriod - std::min<uint16_t>(timer, timerPeriod));
    const float frac = denom > 0.0f ? std::clamp(elapsed / denom, 0.0f, 1.0f) : 0.0f;
    return current + (next - current) * frac;
}

void APU::rebuildTriangleLoudnessTable()
{
    // Equal-loudness contour points used by KYLXBN's NSFPlay fork.
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

// ---- Noise ----
void APU::Noise::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod > 0 ? static_cast<uint16_t>(timerPeriod - 1) : 0;
        const uint16_t feedback = mode ? ((shift >> 6) ^ shift) & 1 : ((shift >> 1) ^ shift) & 1;
        shift = (shift >> 1) | (feedback << 14);
        // Preserve the legacy field in save states, but the actual anti-alias
        // path now comes from CPU-clock integration rather than a one-pole blur.
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

// ---- DMC ----
void APU::Dmc::start()
{
    currentAddr = sampleAddr;
    bytesRemaining = sampleLength;
}

void APU::Dmc::clockTimer()
{
    if (timer == 0) {
        // dmcRates[] is expressed in CPU cycles per output bit. Reloading
        // rate-1 gives an exact interval of `rate` APU::clock() calls.
        timer = rate > 0 ? static_cast<uint16_t>(rate - 1) : 0;

        // When the output unit has exhausted a byte, transfer the sample
        // buffer into the shift register. The memory reader runs separately
        // via Bus DMC DMA and may or may not have produced a byte yet.
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

    // If the memory reader had already committed the reload internally when
    // $4015 disabled the DMC, the transfer still runs as an ordinary reload.
    // AccuracyCoin's explicit-abort X=7 case distinguishes this from the
    // one-cycle stop-bug pulse on the following two positions.
    if (m_dmc.dmaForcedReloadPending) {
        const bool getCycle = m_bus->dmaGetCycle();
        if (!getCycle && m_bus->requestDmcDma(m_dmc.currentAddr, false)) {
            m_dmc.dmaPending = true;
            m_dmc.dmaForcedReloadPending = false;
        }
        return;
    }

    // Explicit-stop bug. The reload request survives the stop only as a
    // one-cycle halt attempt, scheduled on the normal reload PUT phase.
    // Bus suppresses it completely when that halt attempt hits a CPU write.
    if (m_dmc.dmaAbortPending) {
        // The output unit arms an implicit one-cycle pulse during APU::clock().
        // The caller already prevents same-clock submission with
        // deferImplicitAbortSchedule.  Therefore the next scheduler pass is
        // exactly the next CPU bus cycle; do not insert another delay here.

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

    // Load and reload DMC DMAs are scheduled on different DMA phases. A load
    // DMA is created by enabling an idle DMC through $4015. Hardware does not
    // assert RDY immediately: the load request becomes eligible three CPU
    // clocks after the write, then waits for the next GET slot. Depending on
    // the write/APU phase this makes the halt begin 3 or 4 clocks after $4015.
    // Starting on GET also makes the load transfer itself the 3-cycle
    // halt/dummy/get form. A reload DMA is created when the sample buffer
    // empties and targets a PUT slot, where the normal alignment cycle yields
    // the 4-cycle halt/dummy/alignment/get form. If the CPU is
    // writing when RDY is attempted, Bus keeps phase 0 pending and retries on
    // the next CPU slot, so write-delay behavior remains emergent.
    // The three-cycle post-$4015 enable delay gates the memory reader
    // independently of whether the eventual transfer is a load or reload.
    // This distinction matters when $4015 is written while the one-byte
    // sample buffer is already full: the later fetch is reload-timed, but it
    // still cannot execute until the enable latch has propagated.
    if (m_dmc.dmaStartDelay != 0) {
        --m_dmc.dmaStartDelay;
        if (m_dmc.dmaStartDelay != 0)
            return;
    }

    if (m_dmc.sampleBufferFull)
        return;

    const bool getCycle = m_bus->dmaGetCycle();
    // Normal load DMAs are GET-aligned and normal reloads are PUT-aligned.
    // A reload that became necessary before the delayed $4015 enable reached
    // the memory reader is different: its request is already waiting, so the
    // first CPU cycle on which the reader becomes enabled may acquire RDY on
    // either phase. AccuracyCoin DMC tests L/M/N exercise this exact race.
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

        // A one-byte load that completes with the output unit already at the
        // byte-reload boundary is the revision-dependent race.  APU::clock()
        // runs before the DMA GET in the same Bus::clock(), so at GET
        // completion the observable signature for "reload on the next CPU
        // clock" is bitsRemaining==0 && timer==0.  The previous
        // The previous pre-GET phase test described the APU state before the
        // DMA transfer completed and therefore selected the race two CPU
        // clocks too early.
        const bool outputReloadNextClock =
            m_dmc.bitsRemaining == 0 && m_dmc.timer == 0;
        const bool lateUnexpectedReload = oneByteImplicitStop &&
            m_dmcCpuRevision == DmcCpuRevision::Mid1990OrLater &&
            outputReloadNextClock;

        if (lateUnexpectedReload) {
            // Late RP2A03G and RP2A03H contain an additional memory-reader
            // race: if the one-byte load completes on the same APU cycle on
            // which the output unit asks for a reload, the reader performs a
            // full reload DMA from the *same* address on the following PUT
            // slot. Rewind the otherwise-dead reader address so the existing
            // forced-reload path naturally issues that duplicate fetch.
            m_dmc.currentAddr = fetchedAddress;
            m_dmc.dmaForcedReloadPending = true;
            m_dmc.implicitStopWindow = 0;
            traceImplicitDmc(m_bus, "LOAD_COMPLETE_ARM_LATE_RELOAD", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        }
        else if (oneByteImplicitStop && !outputReloadNextClock) {
            // Both early and late 2A03 revisions exhibit the one-cycle
            // aborted-reload bug when the load completes two CPU clocks
            // before the output unit reaches its byte-reload boundary.  The
            // buffer is consumed on the third following APU::clock() call, so
            // keep the opportunity alive for three clocks.  Completion at
            // timer==0 is deliberately excluded: early CPUs perform no extra
            // DMA there, while late CPUs take the full-reload path above.
            m_dmc.implicitStopWindow = 3;
            traceImplicitDmc(m_bus, "LOAD_COMPLETE_ARM_WINDOW", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        }

        if (m_dmc.loop) {
            m_dmc.start();
        }
        else {
            if (m_dmc.irqEnabled)
                m_dmc.irqFlag = true;

            // The normal implicit one-cycle abort is committed later by the
            // output-unit buffer-empty edge. The late-revision same-cycle
            // unexpected reload above is the sole exception because hardware
            // immediately carries that reader request into the next PUT slot.
        }
    }
}

void APU::abortDmcDma()
{
    // No sample byte is transferred. This callback only releases the APU-side
    // in-flight marker after Bus has consumed the single halt cycle.
    m_dmc.dmaPending = false;
}

// ---- Frame ----
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
        // If a length reload is written on the exact half-frame clock, it is
        // ignored when the counter was nonzero. If the counter was zero, the
        // reload is allowed after the clock and the zero counter is not
        // decremented.
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
        // Length-control writes take effect after the frame counter's clock
        // for this CPU cycle, giving the documented one-clock halt delay.
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
    // A delayed $4017 divider reset whose countdown expires on this CPU
    // period becomes visible before the CPU's M2-high bus access. This
    // ordering is observable when a 5-step immediate half-frame clock makes
    // a length counter reach zero on the same CPU period as a $4015 read.
    // Ordinary frame-sequencer events remain post-CPU, as before.
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

    // If the pre-CPU phase consumed the pending reset and the CPU did not
    // write a new $4017 value during this period, do not also advance the new
    // frame sequence here. A new $4017 write installs a fresh countdown and
    // is handled by the normal block below.
    if (resetAppliedPreCpu && m_frameResetDelay == 0) {
        applyPendingLengthWrites();
        return;
    }

    // A $4015 status read requests a frame-IRQ acknowledge, but the latch is
    // actually cleared on the next APU GET phase. Bus::clock invokes this
    // after the CPU access for the same cycle, preserving the observed
    // PUT->GET / GET->PUT double-read asymmetry.
    // AccuracyCoin measures the acknowledge on the PUT->GET transition.
    // Bus::clock() calls this after the CPU bus cycle, so a read performed on
    // the current PUT slot must be cleared now, before a following GET-slot
    // read can observe the latch again. dmaGetCycle() names the *current*
    // bus slot; therefore the transition condition is its inverse.
    const bool enteringGetPhase = m_bus ? !m_bus->dmaGetCycle() : !m_apuPhase;
    if (m_frameIrqClearPending && enteringGetPhase) {
        m_frameIrqFlag = false;
        m_frameIrqClearPending = false;
    }

    // Writes to $4017 do not reset/switch the sequencer on the CPU write
    // cycle. The divider alignment delays that effect by 3 or 4 CPU clocks.
    // Bit 6 (IRQ inhibit) is handled immediately in cpuWrite(); only the mode
    // and sequencer restart wait for this countdown.
    if (m_frameResetDelay != 0) {
        --m_frameResetDelay;
        if (m_frameResetDelay == 0) {
            m_frameMode5 = m_pendingFrameMode5;
            // The externally observed frame sequence is referenced to the
            // $4017 write, while the divider reset itself is delayed by its
            // CPU/APU phase. Seed the elapsed portion without compensating
            // away the real one-clock jitter between the two write phases.
            m_frameCycles = m_pendingFrameStartCycles;

            // Entering 5-step mode generates the initial quarter+half clock
            // at the same delayed instant as the sequencer reset.
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
        // Region-specific 4-step sequence, counted from the delayed $4017 divider reset.
        // The frame IRQ flag rises one clock before the final quarter/half
        // clock, remains asserted across the following clocks, and is cleared
        // only by $4015 read or IRQ-inhibit control.
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
            // The internal frame-IRQ flag pulses on the first two terminal
            // clocks even when IRQ output is inhibited.  Bit 6 of $4017
            // gates the IRQ pin, not these two internal latch-set events.
            m_frameIrqFlag = true;
        }
        else if (m_frameCycles == qh4) {
            quarterFrame();
            halfFrame();
            m_frameIrqFlag = true;
        }
        else if (m_frameCycles == reset) {
            // On the third terminal clock inhibition finally determines the
            // stored flag state.
            m_frameIrqFlag = !m_irqInhibit;
            m_frameCycles = 0;
        }
    }
    else {
        const uint32_t q1 = (m_timing == ConsoleTiming::PAL) ? 8313u : 7457u;
        const uint32_t qh2 = (m_timing == ConsoleTiming::PAL) ? 16627u : 14913u;
        const uint32_t q3 = (m_timing == ConsoleTiming::PAL) ? 24939u : 22371u;
        const uint32_t qh5 = (m_timing == ConsoleTiming::PAL) ? 41561u : 37281u;
        // Region-specific 5-step sequence. The initial quarter+half clock is generated
        // by the delayed $4017 reset above; the ordinary sequence then has a
        // blank fourth step and never generates a frame IRQ.
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
            // The 5-step sequence has a one-CPU-clock-longer wrap interval
            // than a simple zero restart would produce.  Starting at -1
            // (UINT32_MAX) makes the next q1/qh2 events occur 7458/14914
            // clocks after this step-0 event, matching blargg's hardware
            // measurements (including the 52197/52198 third-length edge).
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
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ring[m_ringWrite] = s;
    m_ringWrite = (m_ringWrite + 1) % kRingSize;
    if (m_ringWrite == m_ringRead)
        m_ringRead = (m_ringRead + 1) % kRingSize;
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
        // KYLXBN-style forced linear 2A03 mixing. Pulse gain is the full-scale
        // value of the standard pulse DAC curve; TND gains follow the fork's
        // linearized mixer ratios.
        constexpr float pulseGain = 95.88f / ((8128.0f / 15.0f) + 100.0f);
        constexpr float triangleGain = 0.95f * 45.0f / 208.0f;
        constexpr float noiseGain = 0.95f * 15.0f * 1.95f / 208.0f;
        constexpr float dmcGain = 0.95f * 127.0f / 208.0f;

        // The fork applies loudness compensation around the triangle DAC's
        // midpoint, rather than multiplying a unipolar 0..1 signal directly.
        // KYLXBN's fork leaves periods 0-2 uncompensated. These are already
        // ultrasonic edge cases and applying the equal-loudness table there
        // produces a very large artificial gain.
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

    // Mapper audio is already expressed in the same normalized output domain.
    // Passing the mod flag lets VRC6/VRC7/N163 choose their KYLXBN waveform
    // path while leaving FDS/MMC5/5B behavior unchanged.
    const float expansion = m_cart ? m_cart->expansionAudioSample(m_chipMod) : 0.0f;
    return base + expansion;
}

float APU::filterOutput(float s)
{
    // Remove the cartridge/APU DAC's DC component at the final output stage.
    // 20 Hz is intentionally below musical bass; this is output conditioning,
    // while channel timing/mixing remains in the CPU-clock domain above.
    const double x = s;
    const double y = x - m_hpPrevInput + m_hpCoefficient * m_hpPrevOutput;
    m_hpPrevInput = x;
    m_hpPrevOutput = y;
    // Reserve digital headroom because cartridge expansion audio is summed
    // after the 2A03 DAC model. This is output gain only, not a hardware-level
    // calibration constant; relative channel/chip levels are determined above.
    // Keep enough headroom for loud expansion-audio combinations while avoiding
    // the unnecessarily quiet 0.60 gain used by the previous phase. 0.85 keeps
    // normal 2A03 output materially louder without changing channel balance.
    constexpr double outputHeadroom = 0.85;
    return std::clamp(float(y * outputHeadroom * double(m_masterVolume)), -1.0f, 1.0f);
}

void APU::resetOutputPipeline()
{
    m_sampleTimer = 0.0;
    m_mixAccumulator = 0.0;
    m_mixAccumulatorWeight = 0.0;
    m_hpPrevInput = 0.0;
    m_hpPrevOutput = 0.0;

    std::lock_guard<std::mutex> lock(m_mutex);
    std::fill(m_ring.begin(), m_ring.end(), 0.0f);
    m_ringRead = 0;
    m_ringWrite = 0;
}

void APU::clock()
{
    // Pulse timers run on one half of the APU phase; triangle runs each CPU
    // cycle. Noise/DMC tables here are expressed directly in CPU-cycle periods.
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

    // The DMC stop-DMA bug is tied to the output unit emptying the sample
    // buffer shortly after playback has stopped. A DMA already in flight is
    // not cancelled by $4015; it finishes normally. If the stop occurred in
    // the APU cycle immediately before a reload would have been scheduled,
    // however, the reload survives only as a one-cycle HALT attempt.
    if (dmcBufferEmptied) {
        if (m_dmc.enabled && m_dmc.bytesRemaining != 0 &&
            !m_dmc.dmaLoadPending && m_dmc.dmaStartDelay != 0) {
            m_dmc.dmaReloadWaitingForEnable = true;
        }
        if (m_dmc.sampleLength == 1)
            traceImplicitDmc(m_bus, "BUFFER_EMPTIED", m_dmc.bytesRemaining, m_dmc.sampleLength, m_dmc.enabled, m_dmc.loop, m_dmc.sampleBufferFull, m_dmc.bitsRemaining, m_dmc.timer, m_dmc.implicitStopWindow, m_dmc.dmaAbortPending, m_dmc.dmaForcedReloadPending, m_dmc.dmaPending);
        // The first explicit-stop position is not an aborted one-cycle DMA:
        // the reload has already committed inside the memory reader and still
        // completes as a normal four-cycle transfer. One and two clocks later
        // the surviving request is only the single HALT pulse.
        if (m_dmc.stopBugWindow == 3)
            m_dmc.dmaForcedReloadPending = true;
        else if (m_dmc.stopBugWindow != 0 || m_dmc.implicitStopWindow != 0) {
            // A one-byte implicit abort is latched by the output unit on this
            // clock, but its RDY/HALT attempt is not visible until the next
            // DMA arbitration opportunity.  Do not let the request created
            // here run later in this same Bus::clock() call.  This distinction
            // matters when the following CPU cycle is a JSR stack write: the
            // one-cycle DMA must then be suppressed rather than having already
            // consumed its halt one clock too early.  Explicit $4015-stop
            // aborts keep their established timing.
            const bool implicitAbort = (m_dmc.stopBugWindow == 0 && m_dmc.implicitStopWindow != 0);
            m_dmc.dmaAbortPending = true;
            m_dmc.dmaImplicitAbortPending = implicitAbort;
            // APU::clock() runs before the CPU slot in Bus::clock().  Deferring
            // scheduling for the remainder of this clock is sufficient: the
            // next APU scheduler pass corresponds to the immediately following
            // CPU bus cycle.  No additional delay is required.
            m_dmc.dmaImplicitAbortDelay = 0;
            // The output-unit edge occurs before the CPU bus slot in this
            // Bus::clock(), so the one-cycle RDY pulse is visible immediately
            // on that same CPU cycle. Deferring it would shift the pulse onto
            // the following instruction cycle and break the write-suppression
            // boundary measured by AccuracyCoin.
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

    // Integrate the CPU-clock DAC across the exact host-sample window. A
    // sample boundary usually lands between CPU clocks (~40.58 clocks at
    // 44.1 kHz), so split the current clock fractionally instead of grouping
    // whole clocks into alternating 40/41-clock buckets. This is especially
    // important to the KYLXBN-style averaged-noise path and ultrasonic
    // triangle behavior.
    const double instant = double(mixInstantSample());
    double remaining = 1.0;
    constexpr double epsilon = 1.0e-12;
    while (remaining > epsilon) {
        const double room = std::max(0.0, m_samplePeriod - m_sampleTimer);
        const double weight = std::min(remaining, room);
        if (weight <= epsilon) {
            // Guard against accumulated floating-point error exactly on a
            // boundary. The normal path below immediately resets the window.
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
    std::lock_guard<std::mutex> lock(m_mutex);
    for (int i = 0; i < len; i++) {
        if (m_ringRead != m_ringWrite) {
            stream[i] = m_ring[m_ringRead];
            m_ringRead = (m_ringRead + 1) % kRingSize;
        }
        else stream[i] = 0.0f;
    }
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
        // $4015 does not clear the frame IRQ latch combinationally.  The
        // clear is sampled by the APU on its next GET phase.  This matters
        // for back-to-back reads: a read on PUT followed by one on GET sees
        // the flag set twice, and the GET phase clears it only after the
        // second read has completed (AccuracyCoin Frame Counter IRQ test 7).
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
            // Stopping playback sets bytes_remaining to zero immediately, but
            // does not cancel a DMA that has already been scheduled/acquired.
            // The only post-stop DMA is the documented one-cycle reload pulse
            // when the output unit empties the buffer in the immediately
            // following APU timing window.
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
                // If the sample buffer is already occupied, no load DMA can
                // occur. When the output unit later consumes that byte, the
                // memory reader performs an ordinary reload DMA. Keep the
                // enable delay above, but do not misclassify the future fetch
                // as a GET-aligned load.
                m_dmc.dmaLoadPending = !m_dmc.sampleBufferFull;
            }
        }
        // Writing $4015 clears the DMC IRQ flag
        m_dmc.irqFlag = false;
        break;
    }

    case 0x4017:
        // If an earlier $4017 write's delayed reset is due later in this same
        // CPU clock, do not let this new write overwrite it. Consecutive
        // STA $4017 instructions are four clocks apart and hardware can
        // therefore produce two distinct 5-step immediate half-frame clocks.
        if (m_frameResetDelay == 1) {
            m_frameResetDelay = 0;
            m_frameMode5 = m_pendingFrameMode5;
            m_frameCycles = m_pendingFrameStartCycles;
            if (m_frameMode5) { quarterFrame(); halfFrame(); }
        }

        // IRQ inhibit is immediate: setting bit 6 also acknowledges an
        // already-pending frame IRQ. The sequencer mode/reset, however, is
        // delayed by divider phase. clockFrameCounterPhase() still runs later
        // in this same CPU clock, so seed the countdown one higher to obtain
        // an externally observable 3/4-clock delay after the write.
        m_irqInhibit = (data & 0x40) != 0;
        if (m_irqInhibit)
            m_frameIrqFlag = false;

        m_pendingFrameMode5 = (data & 0x80) != 0;

        // The divider reset/mode switch is visible 3 or 4 CPU clocks after
        // the write. Restore the divider polarity used before Phase 50: that
        // polarity is required by blargg's basic $80 immediate-clock test.
        // clockFrameCounterPhase() decrements the countdown later in this same
        // CPU clock, hence seeds of 4 and 5 produce external delays of 3/4.
        //
        // The actual frame-event epoch is write-relative, however. At the
        // delayed reset, seed m_frameCycles with delay-2 (1 or 2). Combined
        // with the decoded 7457/14913/... counts this yields the hardware-tested
        // write-relative events at 7459/14915/... instead of starting them 3/4
        // clocks late from the divider-reset instant.
        if (m_apuPhase) {
            m_frameResetDelay = 4;
            m_pendingFrameStartCycles = 1;
        }
        else {
            m_frameResetDelay = 5;
            // Do not compensate away the divider's one-clock jitter.  The
            // slower reset phase must leave the subsequent frame events one
            // CPU clock later; blargg's clock_jitter test observes exactly
            // this difference after shifting a $4017 write by one clock.
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
        !get16(m_triangle.timer) || !get16(m_triangle.timerPeriod) ||
        !get8(m_triangle.length) || !get8(m_triangle.linear) ||
        !get8(m_triangle.linearReload) || !getBool(m_triangle.linearReloadFlag) ||
        !get8(m_triangle.sequencer) || !getFloat(m_triangle.phase)) return false;

    if (!getBool(m_noise.enabled) || !getBool(m_noise.lengthHalt) ||
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

    if (!getBool(m_frameMode5) || !getBool(m_irqInhibit) ||
        !getBool(m_frameIrqFlag) || !getBool(m_frameIrqClearPending) || !get32(m_frameCycles) ||
        !getBool(m_apuPhase) || !get8(m_frameResetDelay) ||
        !get8(m_pendingFrameStartCycles) || !getBool(m_pendingFrameMode5) || !getBool(m_chipMod) ||
        !getDouble(m_sampleTimer)) return false;
    if (m_frameResetDelay > 5 || m_pendingFrameStartCycles > 2) return false;

    // Pending length-control/reload flags exist only between a CPU register
    // access and the frame-counter phase of the same Bus clock. Save states
    // are taken between Bus clocks, so never let stale transient flags from
    // the pre-load timeline survive a restore.
    auto clearPendingLength = [](auto& channel) {
        channel.pendingLengthHaltValid = false;
        channel.pendingLengthHalt = false;
        channel.pendingLengthReload = false;
        channel.pendingLengthValue = 0;
    };
    clearPendingLength(m_pulse1);
    clearPendingLength(m_pulse2);
    clearPendingLength(m_triangle);
    clearPendingLength(m_noise);

    // Audio already queued before the load belongs to the old timeline. The
    // mixer integration/filter are host-output state, not emulated hardware,
    // so restart them cleanly while retaining the serialized sample cadence.
    const double restoredSampleTimer = m_sampleTimer;
    resetOutputPipeline();
    m_sampleTimer = restoredSampleTimer;

    return true;
}

