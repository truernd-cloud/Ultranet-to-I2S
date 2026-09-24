/*
* Core 1 continuously reads the samples array, mixes the 16 channels down to one stereo pair,
* and outputs the mix to the I2S stream according to the pico internal clock (output frequency 48KHz)
* The samples array is continually filled by Core 0, synchronised to the incoming Ultranet streams
* This avoids audio sample sync issues as when the output sequence is running ahead, the Core 1 processing
* simply outputs the previous sample a second time, as it's still in the array.
* Or it outputs the new sample, missing one, if it's running behind.
* Provided the pico clock is at a close enough frequency to the incoming Ultranet stream, these corrections
* are infrequent and the samples very similar in amplitude, so there's no audible click or pop.
*
* The level and pan of each channel in the mix are set in the mix_table[] below.
* Edit the table and rebuild to change the mix, or change them at run time via MIDI input.
*
* [한국어]
* 코어1 은 samples 배열을 계속 읽어 16채널을 스테레오 2채널로 믹스하고,
* Pico 내부 클럭(출력 주파수 48kHz)에 맞춰 I2S 로 출력한다.
* samples 배열은 코어0 이 Ultranet 입력 스트림에 동기되어 계속 채운다.
* 출력이 입력보다 빠르면 코어1 은 배열에 남아 있는 이전 샘플을 한 번 더 출력하고,
* 느리면 새 샘플을 출력하면서 하나를 건너뛴다. 이렇게 해서 두 클럭 간 동기 문제를 피한다.
* Pico 클럭이 Ultranet 클럭과 충분히 가까우면 이런 보정은 드물게 일어나고,
* 인접 샘플의 크기가 거의 같으므로 귀에 들리는 클릭/팝 노이즈는 생기지 않는다.
*
* 채널별 믹스 레벨과 팬은 아래 mix_table[] 에서 설정한다.
* 믹스를 바꾸려면 테이블을 수정하고 다시 빌드하거나, 동작 중에 MIDI 입력으로 바꾼다.
*/

#include <math.h>
#include "ultranet.h"

/*
* Mix settings for each channel
*  gain_db : channel level in dB (0.0 = unity, maximum +6.0, MUTE = off)
*  pan     : -1.0 = full left, 0.0 = centre, +1.0 = full right
* Pan uses a balance law: a centred channel goes to both sides at full level,
* panning towards one side lowers only the opposite side.
*
* [한국어] 채널별 믹스 설정
*  gain_db : 채널 레벨(dB). 0.0 = 원래 크기, 최대 +6.0, MUTE = 끔
*  pan     : -1.0 = 완전히 왼쪽, 0.0 = 가운데, +1.0 = 완전히 오른쪽
* 팬은 밸런스 방식이다: 가운데 채널은 양쪽에 원래 크기로 들어가고,
* 한쪽으로 팬하면 반대쪽 레벨만 줄어든다.
* 16채널을 합치므로 기본값은 클리핑 여유를 위해 -12dB 로 두었다. 합이 풀스케일을 넘으면 클리핑된다.
*/
#define MUTE (-200.0f)                                      // gain_db value for a muted channel   (채널 끄기)

typedef struct
{
    float gain_db;                                          // channel level in dB       (채널 레벨, dB)
    float pan;                                              // -1.0 left .. +1.0 right   (-1.0 왼쪽 .. +1.0 오른쪽)
} mix_channel_t;

static const mix_channel_t mix_table[UNET_CHANNELS] =
{
    //  gain_db   pan               channel   (채널)
    { -12.0f,   0.0f },         //  1
    { -12.0f,   0.0f },         //  2
    { -12.0f,   0.0f },         //  3
    { -12.0f,   0.0f },         //  4
    { -12.0f,   0.0f },         //  5
    { -12.0f,   0.0f },         //  6
    { -12.0f,   0.0f },         //  7
    { -12.0f,   0.0f },         //  8
    { -12.0f,   0.0f },         //  9
    { -12.0f,   0.0f },         // 10
    { -12.0f,   0.0f },         // 11
    { -12.0f,   0.0f },         // 12
    { -12.0f,   0.0f },         // 13
    { -12.0f,   0.0f },         // 14
    { -12.0f,   0.0f },         // 15
    { -12.0f,   0.0f },         // 16
};

// Mix gains are fixed point with MIX_GAIN_BITS fractional bits (512 = unity gain)
// Samples are used as 22 bit signed values, so sample x gain (max 2.0 = 1024) fits in 32 bits
// 믹스 게인은 소수부 MIX_GAIN_BITS 비트의 고정소수점이다 (512 = 원래 크기).
// 샘플은 22비트 부호 있는 값으로 쓰므로, 샘플 x 게인(최대 2.0 = 1024)이 32비트 안에 들어간다.
#define MIX_GAIN_BITS 9
#define MIX_GAIN_MAX (2 << MIX_GAIN_BITS)                   // +6dB   (최대 게인)
#define SAMPLE_SHIFT 10                                     // 32 bit left aligned -> 22 bit signed   (32비트 왼쪽 정렬 → 22비트 부호 있는 값)
#define SAMPLE_MAX ((1 << 21) - 1)                          // largest 22 bit signed value            (22비트 최댓값)
#define SAMPLE_MIN (-(1 << 21))                             // smallest 22 bit signed value           (22비트 최솟값)

#define PAN_RANGE 63                                        // pan steps each side of centre (-63 .. +63)   (가운데 기준 한쪽 팬 단계 수)

static int32_t chan_vol[UNET_CHANNELS];                     // level per channel (fixed point)       (채널별 레벨, 고정소수점)
static int32_t chan_pan[UNET_CHANNELS];                     // pan per channel, -63 left .. +63 right   (채널별 팬, -63 왼쪽 .. +63 오른쪽)
static int32_t gain_l[UNET_CHANNELS];                       // left gain per channel (fixed point)   (채널별 왼쪽 게인, 고정소수점)
static int32_t gain_r[UNET_CHANNELS];                       // right gain per channel (fixed point)  (채널별 오른쪽 게인, 고정소수점)

// convert a linear gain to fixed point, limited to MIX_GAIN_MAX
// 선형 게인을 고정소수점으로 바꾼다 (MIX_GAIN_MAX 로 제한)
static int32_t gain_to_fixed(float gain)
{
    int32_t fixed = (int32_t)(gain * (1 << MIX_GAIN_BITS) + 0.5f);
    return (fixed > MIX_GAIN_MAX) ? MIX_GAIN_MAX : fixed;
}

// recalculate left/right gains of one channel from its level and pan (integer only, cheap enough for MIDI updates)
// 채널 하나의 레벨과 팬으로 왼쪽/오른쪽 게인을 다시 계산한다 (정수 연산만 사용하므로 MIDI 로 바꿀 때도 빠름)
static void mix_update_channel(int ch)
{
    int32_t vol = chan_vol[ch];
    int32_t pan = chan_pan[ch];
    gain_l[ch] = (pan > 0) ? vol * (PAN_RANGE - pan) / PAN_RANGE : vol;   // panning right lowers left   (오른쪽으로 팬하면 왼쪽 감소)
    gain_r[ch] = (pan < 0) ? vol * (PAN_RANGE + pan) / PAN_RANGE : vol;   // panning left lowers right   (왼쪽으로 팬하면 오른쪽 감소)
}

// calculate channel levels, pans and gains from the mix table (once at startup)
// 믹스 테이블에서 채널별 레벨, 팬, 게인을 계산한다 (시작 시 한 번)
static void mix_init(void)
{
    for(int count = 0; count < UNET_CHANNELS; count++)
    {
        float pan = mix_table[count].pan;
        if(pan < -1.0f) pan = -1.0f;
        if(pan > 1.0f) pan = 1.0f;
        chan_vol[count] = gain_to_fixed(powf(10.0f, mix_table[count].gain_db / 20.0f));  // dB -> linear -> fixed point   (dB → 선형 → 고정소수점)
        chan_pan[count] = (int32_t)lroundf(pan * PAN_RANGE);
        mix_update_channel(count);
    }
}

// scale a mixed sum back to 22 bits, clip it, and left align it into a 32 bit I2S word
// 믹스 합계를 22비트로 되돌리고 클리핑한 뒤, 32비트 I2S 워드로 왼쪽 정렬한다
static inline uint32_t mix_to_i2s(int64_t acc)
{
    int64_t out = acc >> MIX_GAIN_BITS;
    if(out > SAMPLE_MAX) out = SAMPLE_MAX;                  // clip at full scale   (풀스케일에서 클리핑)
    if(out < SAMPLE_MIN) out = SAMPLE_MIN;
    return (uint32_t)(int32_t)out << SAMPLE_SHIFT;
}

// mix all channels to one stereo pair (I2S words)
// 모든 채널을 스테레오 한 쌍(I2S 워드)으로 믹스한다
static void mix_samples(uint32_t *left, uint32_t *right)
{
    int64_t acc_l = 0, acc_r = 0;                           // mix sums   (믹스 합계)
    for(int count = 0; count < UNET_CHANNELS; count++)
    {
        int32_t sample = (int32_t)samples[count] >> SAMPLE_SHIFT;   // 22 bit signed sample   (22비트 부호 있는 샘플)
        acc_l += sample * gain_l[count];                    // product fits in 32 bits   (곱은 32비트 안에 들어감)
        acc_r += sample * gain_r[count];
    }
    *left = mix_to_i2s(acc_l);
    *right = mix_to_i2s(acc_r);
}

#ifdef MIDI
/*
* MIDI input (UART) for remote control of the mix
* MIDI channel 1-16 controls mixer channel 1-16:
*  CC 7  (volume) : 0 = off, 127 = unity (0dB), curve = (value/127)^2 (40 x log10 law, as General MIDI)
*  CC 10 (pan)    : 0 = full left, 64 = centre, 127 = full right
* Other messages are ignored. Running status is supported.
* A MIDI setting replaces the value from mix_table[] until the next reset.
*
* [한국어] 믹스 원격 조작용 MIDI 입력 (UART)
* MIDI 채널 1~16 이 믹서 채널 1~16 을 조작한다:
*  CC 7  (볼륨) : 0 = 끔, 127 = 원래 크기(0dB), 곡선 = (값/127)^2 (General MIDI 와 같은 40 x log10 곡선)
*  CC 10 (팬)   : 0 = 완전히 왼쪽, 64 = 가운데, 127 = 완전히 오른쪽
* 다른 메시지는 무시한다. 러닝 스테이터스를 지원한다.
* MIDI 로 바꾼 값은 다음 리셋 전까지 mix_table[] 의 값을 대신한다.
*/
#define MIDI_CC_VOLUME 7                                    // controller number for volume   (볼륨 컨트롤러 번호)
#define MIDI_CC_PAN 10                                      // controller number for pan      (팬 컨트롤러 번호)

static uint8_t midi_status;                                 // current (running) status byte, 0 = none   (현재 스테이터스 바이트, 0 = 없음)
static uint8_t midi_data[2];                                // data bytes of current message              (현재 메시지의 데이터 바이트)
static uint midi_count;                                     // data bytes received so far                 (지금까지 받은 데이터 바이트 수)

// set up the UART for MIDI input (31250 baud, 8N1, receive only)
// MIDI 입력용 UART 를 설정한다 (31250 baud, 8N1, 수신만)
static void midi_init(void)
{
    uart_init(MIDI_UART, MIDI_BAUD);                        // baud rate from current clk_peri       (현재 주변장치 클럭 기준으로 속도 설정)
    gpio_set_function(MIDI_RX_PIN, GPIO_FUNC_UART);         // connect RX pin to UART                (RX 핀을 UART 에 연결)
    gpio_pull_up(MIDI_RX_PIN);                              // idle high if nothing is connected     (연결이 없을 때 HIGH(유휴) 유지)
}

// handle a complete Control Change message
// 완성된 컨트롤 체인지 메시지를 처리한다
static void midi_control_change(uint ch, uint controller, uint value)
{
    if(controller == MIDI_CC_VOLUME)
    {
        chan_vol[ch] = (int32_t)((value * value * (1 << MIX_GAIN_BITS) + 8064) / 16129);  // (value/127)^2, rounded   (반올림)
        mix_update_channel(ch);
    }
    else if(controller == MIDI_CC_PAN)
    {
        int32_t pan = (int32_t)value - 64;                  // 64 = centre   (64 = 가운데)
        chan_pan[ch] = (pan < -PAN_RANGE) ? -PAN_RANGE : pan;
        mix_update_channel(ch);
    }
}

// process one byte received from MIDI input
// MIDI 입력에서 받은 바이트 하나를 처리한다
static void midi_byte(uint8_t byte)
{
    if(byte >= 0xF8)                                        // real time messages can appear anywhere - ignore   (리얼타임 메시지는 어디든 끼어들 수 있음 - 무시)
        return;
    if(byte & 0x80)                                         // status byte   (스테이터스 바이트)
    {
        // channel messages set running status, system messages (sysex etc.) cancel it
        // 채널 메시지는 러닝 스테이터스가 되고, 시스템 메시지(시스템 익스클루시브 등)는 이를 취소한다
        midi_status = (byte < 0xF0) ? byte : 0;
        midi_count = 0;
        return;
    }
    if(midi_status == 0)                                    // data byte without a status (e.g. sysex data) - ignore   (스테이터스 없는 데이터 - 무시)
        return;

    midi_data[midi_count++] = byte;
    uint type = midi_status & 0xF0;
    uint needed = (type == 0xC0 || type == 0xD0) ? 1 : 2;   // program change and channel pressure have one data byte   (1바이트 메시지)
    if(midi_count < needed)
        return;
    midi_count = 0;                                         // message complete, keep status for running status   (메시지 완성, 러닝 스테이터스 유지)

    if(type == 0xB0)                                        // Control Change   (컨트롤 체인지)
        midi_control_change(midi_status & 0x0F, midi_data[0], midi_data[1]);
}

// read all bytes waiting in the UART receive FIFO
// UART 수신 FIFO 에 쌓인 바이트를 모두 처리한다
static inline void midi_poll(void)
{
    while(uart_is_readable(MIDI_UART))
        midi_byte((uint8_t)uart_getc(MIDI_UART));
}
#endif // MIDI

#ifdef MCLK
// I2S MCLK 출력용 PIO 상태 머신을 설정한다 (PIO 클럭 = 시스템 클럭 / AUDIV, 2사이클마다 한 주기 → 약 12.288MHz = 256fs)
void mclk_pio_init(PIO pio, uint sm, uint pin)
{
    uint offset = pio_add_program(pio, &mclk_program);      // load clock code                        (클럭 프로그램 적재)
    pio_sm_config c = mclk_program_get_default_config(offset);  // get default structure             (기본 설정 구조체)
    pio_gpio_init(pio, pin);                                // iniitialise pin as pio output          (핀을 PIO 출력으로 초기화)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // set pin direction to output            (핀 방향을 출력으로)
    sm_config_set_clkdiv_int_frac(&c, AUDIV, 0);
    sm_config_set_set_pins (&c, pin, 1);                    // output pin range base and count        (set 명령용 출력 핀 기준과 개수)
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine       (상태 머신에 설정 적용)
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running            (상태 머신 시작)
}
#endif // MCLK

// I2S 출력용 PIO 상태 머신을 설정한다 (아직 시작하지 않음 - FIFO 를 채운 뒤 core1_entry 에서 시작)
// 핀 배치: pin = DATA, pin+1 = BCLK, pin+2 = LRCLK
void i2s_pio_init(PIO pio, uint sm, uint pin, uint offset)
{
    pio_sm_config c = i2s_program_get_default_config(offset);  // get default structure             (기본 설정 구조체)
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);  // set base+3 pins to output              (기준 핀부터 3개를 출력으로)
    sm_config_set_clkdiv_int_frac(&c, AUDIV, 0);            // set frequency of I2S SM to fs x 512    (PIO 클럭 분주 설정)
    sm_config_set_out_pins (&c, pin, 3);                    // out pin range base and count           (out 명령용 핀 기준과 개수)
    sm_config_set_sideset_pins (&c, pin+1);                 // sideset pin range base                 (사이드셋 핀 기준 = BCLK)
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // configure 8 depth output fifo          (TX FIFO 8단)
    sm_config_set_out_shift(&c, false, false, 32);          // set shift left, no autpull for out FIFO (왼쪽 시프트(MSB 먼저), 자동 pull 없음)
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine       (상태 머신에 설정 적용)
}

// 코어1 이 사용하는 핀(I2S)을 picotool 용 바이너리 정보로 등록한다
void set_core1_info(void)
{
    bi_decl(bi_pin_mask_with_name(((1<<I2S_PINS)|(1<<(I2S_PINS+1))|(1<<(I2S_PINS+2))), "I2S DATA,BCLK,LRCLK (16ch mix)"));
#ifdef MIDI
    bi_decl(bi_1pin_with_name(MIDI_RX_PIN, "MIDI In (UART RX)"));
#endif // MIDI
}


// 코어1 진입점: 믹스 게인과 출력 하드웨어를 초기화한 뒤, 16채널 믹스를 I2S 로 끝없이 출력한다
void core1_entry(void)                                      // Core1 starts executiing here
{
    uint i2s_offset;                                        // position for i2s code in pio           (PIO 안의 I2S 코드 위치)
    uint32_t left, right;                                   // mixed output samples                   (믹스된 출력 샘플)

    mix_init();                                             // calculate mix gains from mix table     (믹스 테이블로 게인 계산)
#ifdef MIDI
    midi_init();                                            // start MIDI input                       (MIDI 입력 시작)
#endif // MIDI

#ifdef MCLK
    mclk_pio_init(MCLK_PIO, MCLK_SM, MCLK_PIN);             // start I2S MCLK                         (I2S MCLK 출력 시작)
#endif // MCLK

    i2s_offset = pio_add_program(I2S_PIO, &i2s_program);    // load i2s output code                   (I2S 프로그램 적재)
    i2s_pio_init(I2S_PIO, I2S_SM, I2S_PINS, i2s_offset);
    sleep_ms(200);                                          // wait for incoming samples to start     (입력 샘플이 들어오기 시작할 때까지 대기)
    // ensure there is data in the output FIFO before starting the state machine
    // 상태 머신을 시작하기 전에 출력 FIFO 에 데이터를 미리 넣어 둔다 (왼쪽, 오른쪽 순)
    mix_samples(&left, &right);
    pio_sm_put_blocking(I2S_PIO, I2S_SM, left);
    pio_sm_put_blocking(I2S_PIO, I2S_SM, right);

    pio_sm_set_enabled(I2S_PIO, I2S_SM, true);              // start I2S state machine               (I2S 상태 머신 시작)

    // 메인 출력 루프. pio_sm_put_blocking 은 FIFO 에 빈 자리가 생길 때까지 기다리므로
    // 루프 속도는 I2S 출력 클럭(48kHz)에 자동으로 맞춰진다. 믹스 계산은 샘플 주기(약 20.8us)보다 훨씬 짧다.
    while(true)                                             // output samples synchronised with I2S stream
    {
#ifdef MIDI
        midi_poll();                                        // apply any MIDI changes to the mix      (MIDI 로 받은 믹스 변경 적용)
#endif // MIDI
        mix_samples(&left, &right);                         // mix 16 channels to stereo              (16채널을 스테레오로 믹스)
        pio_sm_put_blocking(I2S_PIO, I2S_SM, left);         // left channel                           (왼쪽 채널)
        pio_sm_put_blocking(I2S_PIO, I2S_SM, right);        // right channel                          (오른쪽 채널)
    }
}
