#include "APU.hpp"
#include <SDL.h>
#include <cstring>
#include <algorithm>
#include <cmath>

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

const uint16_t APU::noisePeriods[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
};

const uint8_t APU::triangleSequence[32] = {
     15,14,13,12,11,10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
      0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15
};

static void sdlAudioCallback(void* userdata, Uint8* stream, int len)
{
    APU* apu = static_cast<APU*>(userdata);
    float* out = reinterpret_cast<float*>(stream);
    int samples = len / (int)sizeof(float);
    apu->fillBuffer(out, samples);
}

APU::APU()
{
    m_ring.assign(kRingSize, 0.0f);
}

APU::~APU()
{
    shutdownAudio();
}

bool APU::initAudio()
{
    SDL_AudioSpec want{}, have{};
    want.freq = kSampleRate;
    want.format = AUDIO_F32SYS;
    want.channels = 1;
    want.samples = 512;
    want.callback = sdlAudioCallback;
    want.userdata = this;

    if (SDL_OpenAudio(&want, &have) < 0)
        return false;
    SDL_PauseAudio(0);
    return true;
}

void APU::shutdownAudio()
{
    SDL_CloseAudio();
}

// ---- Pulse ----
void APU::Pulse::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod;
        dutyBit = (dutyBit + 1) & 7;
    }
    else {
        timer--;
    }
}

void APU::Pulse::clockEnvelope()
{
    if (envelopeStart) {
        envelope = 15;
        envelopePeriod = volume;
        envelopeStart = false;
    }
    else if (envelopePeriod == 0) {
        envelopePeriod = volume;
        if (envelope > 0)
            envelope--;
        else if (lengthHalt)
            envelope = 15;
    }
    else {
        envelopePeriod--;
    }
}

void APU::Pulse::clockLength()
{
    if (!lengthHalt && length > 0)
        length--;
}

void APU::Pulse::clockSweep(bool isPulse1)
{
    if (sweepDivider == 0) {
        if (sweepEnabled && sweepShift > 0 && timerPeriod >= 8) {
            int delta = timerPeriod >> sweepShift;
            if (sweepNegate)
                delta = isPulse1 ? -delta - 1 : -delta;
            int target = (int)timerPeriod + delta;
            if (target >= 0 && target <= 0x7FF)
                timerPeriod = (uint16_t)target;
        }
        sweepDivider = sweepPeriod;
    }
    else {
        sweepDivider--;
    }
    if (sweepReload) {
        sweepDivider = sweepPeriod;
        sweepReload = false;
    }
}

float APU::Pulse::sample() const
{
    if (!enabled || length == 0 || timerPeriod < 8 || timerPeriod > 0x7FF)
        return 0.0f;
    if (!dutyTable[duty][dutyBit])
        return 0.0f;
    return constant ? (volume / 15.0f) : (envelope / 15.0f);
}

// ---- Triangle ----
void APU::Triangle::clockTimer(bool chipMod)
{
    if (timer == 0) {
        timer = timerPeriod;
        if (length > 0 && linear > 0) {
            if (chipMod) {
                // Advance continuous phase: one full triangle cycle every 32 sequencer steps
                // Hardware clocks sequencer once per timer expiry → 32 steps = 1 period of wave
                phase += 1.0f / 32.0f;
                if (phase >= 1.0f)
                    phase -= 1.0f;
                sequencer = (sequencer + 1) & 31; // keep in sync for mode switches
            }
            else {
                sequencer = (sequencer + 1) & 31;
            }
        }
    }
    else {
        timer--;
    }
}

void APU::Triangle::clockLinear()
{
    if (linearReloadFlag)
        linear = linearReload;
    else if (linear > 0)
        linear--;
    if (!lengthHalt)
        linearReloadFlag = false;
}

void APU::Triangle::clockLength()
{
    if (!lengthHalt && length > 0)
        length--;
}

float APU::Triangle::sample(bool chipMod) const
{
    if (!enabled || length == 0 || linear == 0 || timerPeriod < 2)
        return 0.0f;

    if (chipMod) {
        // Pure triangle: 0→1→0 over phase [0,1)
        float p = phase;
        float tri = (p < 0.5f) ? (p * 2.0f) : (2.0f - p * 2.0f);
        return tri; // 0..1
    }
    return triangleSequence[sequencer] / 15.0f;
}

// Approximate ISO 226-inspired boost for low triangle notes (kylxbn-style)
float APU::triangleLoudnessGain(uint16_t period) const
{
    // Lower pitch (higher period) → more gain, capped
    // period ~ 0x7FF is very low; ~32 is high
    float t = (float)period / 2047.0f; // 0..1
    float gain = 1.0f + t * 1.25f;     // up to ~2.25x on lowest notes
    return std::min(gain, 2.5f);
}

// ---- Noise ----
void APU::Noise::clockTimer(bool chipMod)
{
    if (timer == 0) {
        timer = timerPeriod;
        uint16_t feedback = mode ? ((shift >> 6) ^ (shift >> 0)) & 1
            : ((shift >> 1) ^ (shift >> 0)) & 1;
        shift = (shift >> 1) | (feedback << 14);
        if (chipMod) {
            // Light float smoothing toward the new bit
            float target = (shift & 1) ? 0.0f : 1.0f;
            smooth += (target - smooth) * 0.35f;
        }
    }
    else {
        timer--;
    }
}

void APU::Noise::clockEnvelope()
{
    if (envelopeStart) {
        envelope = 15;
        envelopePeriod = volume;
        envelopeStart = false;
    }
    else if (envelopePeriod == 0) {
        envelopePeriod = volume;
        if (envelope > 0) envelope--;
        else if (lengthHalt) envelope = 15;
    }
    else {
        envelopePeriod--;
    }
}

void APU::Noise::clockLength()
{
    if (!lengthHalt && length > 0)
        length--;
}

float APU::Noise::sample(bool chipMod) const
{
    if (!enabled || length == 0)
        return 0.0f;
    float vol = constant ? (volume / 15.0f) : (envelope / 15.0f);
    if (chipMod)
        return smooth * vol;
    if (shift & 1)
        return 0.0f;
    return vol;
}

// ---- Frame counter ----
void APU::quarterFrame()
{
    m_pulse1.clockEnvelope();
    m_pulse2.clockEnvelope();
    m_noise.clockEnvelope();
    m_triangle.clockLinear();
}

void APU::halfFrame()
{
    m_pulse1.clockLength();
    m_pulse2.clockLength();
    m_noise.clockLength();
    m_triangle.clockLength();
    m_pulse1.clockSweep(true);
    m_pulse2.clockSweep(false);
}

void APU::clockFrameCounter()
{
    static const uint32_t steps4[] = { 3729, 7457, 11186, 14915 };
    static const uint32_t steps5[] = { 3729, 7457, 11186, 18641 };

    m_frameCycles++;
    const uint32_t* steps = m_frameMode5 ? steps5 : steps4;

    for (int i = 0; i < 4; i++) {
        if (m_frameCycles == steps[i]) {
            quarterFrame();
            if (i == 1 || i == 3 || (m_frameMode5 && i == 0))
                halfFrame();
            if (i == 3)
                m_frameCycles = 0;
        }
    }
}

void APU::pushSample(float s)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ring[m_ringWrite] = s;
    m_ringWrite = (m_ringWrite + 1) % kRingSize;
    if (m_ringWrite == m_ringRead)
        m_ringRead = (m_ringRead + 1) % kRingSize;
}

float APU::mixSample() const
{
    float p1 = m_pulse1.sample();
    float p2 = m_pulse2.sample();
    float t = m_triangle.sample(m_chipMod);
    float n = m_noise.sample(m_chipMod);

    if (m_chipMod) {
        // Linear float mix (kylxbn): channels don't steal volume from each other
        if (m_chipMod)
            t *= triangleLoudnessGain(m_triangle.timerPeriod);

        float s = p1 * 0.15f + p2 * 0.15f + t * 0.20f + n * 0.12f;
        return std::clamp(s * 1.8f, -1.0f, 1.0f);
    }

    // Hardware-ish nonlinear mix
    float pulseOut = 0.0f;
    if (p1 + p2 > 0.0f)
        pulseOut = 95.88f / ((8128.0f / (p1 * 15.0f + p2 * 15.0f)) + 100.0f);

    float tnd = 0.0f;
    float tndSum = t * 15.0f / 8227.0f + n * 15.0f / 12241.0f;
    if (tndSum > 0.0f)
        tnd = 159.79f / (1.0f / tndSum + 100.0f);

    return std::clamp((pulseOut + tnd) * 1.5f, -1.0f, 1.0f);
}

void APU::clock()
{
    m_pulse1.clockTimer();
    m_pulse2.clockTimer();
    m_triangle.clockTimer(m_chipMod);
    m_noise.clockTimer(m_chipMod);
    clockFrameCounter();

    m_sampleTimer += 1.0;
    if (m_sampleTimer >= m_samplePeriod) {
        m_sampleTimer -= m_samplePeriod;
        pushSample(mixSample());
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
        else {
            stream[i] = 0.0f;
        }
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
        return v;
    }
    return 0;
}

void APU::cpuWrite(uint16_t addr, uint8_t data)
{
    switch (addr) {
    case 0x4000:
        m_pulse1.duty = (data >> 6) & 3;
        m_pulse1.lengthHalt = (data & 0x20) != 0;
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
        m_pulse1.length = lengthTable[(data >> 3) & 0x1F];
        m_pulse1.dutyBit = 0;
        m_pulse1.envelopeStart = true;
        break;

    case 0x4004:
        m_pulse2.duty = (data >> 6) & 3;
        m_pulse2.lengthHalt = (data & 0x20) != 0;
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
        m_pulse2.length = lengthTable[(data >> 3) & 0x1F];
        m_pulse2.dutyBit = 0;
        m_pulse2.envelopeStart = true;
        break;

    case 0x4008:
        m_triangle.lengthHalt = (data & 0x80) != 0;
        m_triangle.linearReload = data & 0x7F;
        break;
    case 0x400A:
        m_triangle.timerPeriod = (m_triangle.timerPeriod & 0xFF00) | data;
        break;
    case 0x400B:
        m_triangle.timerPeriod = (m_triangle.timerPeriod & 0x00FF) | ((data & 7) << 8);
        m_triangle.length = lengthTable[(data >> 3) & 0x1F];
        m_triangle.linearReloadFlag = true;
        break;

    case 0x400C:
        m_noise.lengthHalt = (data & 0x20) != 0;
        m_noise.constant = (data & 0x10) != 0;
        m_noise.volume = data & 0x0F;
        break;
    case 0x400E:
        m_noise.mode = (data & 0x80) != 0;
        m_noise.timerPeriod = noisePeriods[data & 0x0F];
        break;
    case 0x400F:
        m_noise.length = lengthTable[(data >> 3) & 0x1F];
        m_noise.envelopeStart = true;
        break;

    case 0x4015:
        m_pulse1.enabled = (data & 0x01) != 0;
        m_pulse2.enabled = (data & 0x02) != 0;
        m_triangle.enabled = (data & 0x04) != 0;
        m_noise.enabled = (data & 0x08) != 0;
        if (!m_pulse1.enabled) m_pulse1.length = 0;
        if (!m_pulse2.enabled) m_pulse2.length = 0;
        if (!m_triangle.enabled) m_triangle.length = 0;
        if (!m_noise.enabled) m_noise.length = 0;
        break;

    case 0x4017:
        m_frameMode5 = (data & 0x80) != 0;
        m_irqInhibit = (data & 0x40) != 0;
        m_frameCycles = 0;
        if (m_frameMode5) {
            quarterFrame();
            halfFrame();
        }
        break;
    }
}

void APU::saveState(std::vector<uint8_t>& out) const
{
    auto put8 = [&](uint8_t v) { out.push_back(v); };
    auto put16 = [&](uint16_t v) { out.push_back(v & 0xFF); out.push_back((v >> 8) & 0xFF); };

    put8(m_pulse1.enabled); put8(m_pulse1.length); put16(m_pulse1.timerPeriod); put8(m_pulse1.volume);
    put8(m_pulse2.enabled); put8(m_pulse2.length); put16(m_pulse2.timerPeriod); put8(m_pulse2.volume);
    put8(m_triangle.enabled); put8(m_triangle.length); put16(m_triangle.timerPeriod); put8(m_triangle.linear);
    put8(m_noise.enabled); put8(m_noise.length); put16(m_noise.timerPeriod); put8(m_noise.volume);
    put8(m_frameMode5); put8(m_irqInhibit);
    put8(m_chipMod ? 1 : 0);
}

bool APU::loadState(const uint8_t*& p, const uint8_t* end)
{
    auto get8 = [&](uint8_t& v) -> bool { if (p >= end) return false; v = *p++; return true; };
    auto get16 = [&](uint16_t& v) -> bool {
        if (p + 2 > end) return false; v = p[0] | (uint16_t(p[1]) << 8); p += 2; return true;
        };
    uint8_t b = 0;
    if (!get8(b)) return false; m_pulse1.enabled = b;
    if (!get8(m_pulse1.length) || !get16(m_pulse1.timerPeriod) || !get8(m_pulse1.volume)) return false;
    if (!get8(b)) return false; m_pulse2.enabled = b;
    if (!get8(m_pulse2.length) || !get16(m_pulse2.timerPeriod) || !get8(m_pulse2.volume)) return false;
    if (!get8(b)) return false; m_triangle.enabled = b;
    if (!get8(m_triangle.length) || !get16(m_triangle.timerPeriod) || !get8(m_triangle.linear)) return false;
    if (!get8(b)) return false; m_noise.enabled = b;
    if (!get8(m_noise.length) || !get16(m_noise.timerPeriod) || !get8(m_noise.volume)) return false;
    if (!get8(b)) return false; m_frameMode5 = b;
    if (!get8(b)) return false; m_irqInhibit = b;
    if (p < end) {
        if (!get8(b)) return false;
        m_chipMod = b != 0;
    }
    return true;
}




