#include "APU.hpp"
#include <SDL.h>
#include <cstring>
#include <algorithm>

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
    float vol = constant ? (volume / 15.0f) : (envelope / 15.0f);
    return vol;
}

// ---- Triangle ----
void APU::Triangle::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod;
        if (length > 0 && linear > 0)
            sequencer = (sequencer + 1) & 31;
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

float APU::Triangle::sample() const
{
    if (!enabled || length == 0 || linear == 0 || timerPeriod < 2)
        return 0.0f;
    return triangleSequence[sequencer] / 15.0f;
}

// ---- Noise ----
void APU::Noise::clockTimer()
{
    if (timer == 0) {
        timer = timerPeriod;
        uint16_t feedback = mode ? ((shift >> 6) ^ (shift >> 0)) & 1
            : ((shift >> 1) ^ (shift >> 0)) & 1;
        shift = (shift >> 1) | (feedback << 14);
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

float APU::Noise::sample() const
{
    if (!enabled || length == 0 || (shift & 1))
        return 0.0f;
    float vol = constant ? (volume / 15.0f) : (envelope / 15.0f);
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
    // 4-step: 3728.5, 7456.5, 11185.5, 14914.5 CPU cycles (approx integers)
    static const uint32_t steps4[] = { 3729, 7457, 11186, 14915 };
    static const uint32_t steps5[] = { 3729, 7457, 11186, 18641 };

    m_frameCycles++;
    const uint32_t* steps = m_frameMode5 ? steps5 : steps4;
    int n = m_frameMode5 ? 4 : 4;

    for (int i = 0; i < n; i++) {
        if (m_frameCycles == steps[i]) {
            quarterFrame();
            if (i == 1 || i == 3 || (m_frameMode5 && i == 0))
                halfFrame();
            if (!m_frameMode5 && i == 3)
                m_frameCycles = 0;
            if (m_frameMode5 && i == 3)
                m_frameCycles = 0;
        }
    }
}

void APU::pushSample(float s)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_ring[m_ringWrite] = s;
    m_ringWrite = (m_ringWrite + 1) % kRingSize;
    // drop if overflow
    if (m_ringWrite == m_ringRead)
        m_ringRead = (m_ringRead + 1) % kRingSize;
}

void APU::clock()
{
    m_pulse1.clockTimer();
    m_pulse2.clockTimer();
    // triangle clocks at 2x in hardware relative to some refs – clock every CPU cycle is fine enough
    m_triangle.clockTimer();
    m_noise.clockTimer();
    clockFrameCounter();

    m_sampleTimer += 1.0;
    if (m_sampleTimer >= m_samplePeriod) {
        m_sampleTimer -= m_samplePeriod;
        float s = 0.0f;
        s += 0.00752f * 95.88f * (m_pulse1.sample() + m_pulse2.sample()); // rough mix
        // Better nonlinear mix approximation
        float p1 = m_pulse1.sample();
        float p2 = m_pulse2.sample();
        float t = m_triangle.sample();
        float n = m_noise.sample();

        float pulseOut = 0.0f;
        if (p1 + p2 > 0.0f)
            pulseOut = 95.88f / ((8128.0f / (p1 * 15.0f + p2 * 15.0f)) + 100.0f);

        float tnd = 0.0f;
        float tndSum = t * 15.0f / 8227.0f + n * 15.0f / 12241.0f;
        if (tndSum > 0.0f)
            tnd = 159.79f / (1.0f / tndSum + 100.0f);

        s = pulseOut + tnd;
        // scale
        s = std::clamp(s * 1.5f, -1.0f, 1.0f);
        pushSample(s);
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

uint8_t APU::cpuRead(uint16_t addr)
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
        // Pulse 1
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

        // Pulse 2
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

        // Triangle
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

        // Noise
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

        // Status
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

        // Frame counter
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

