/*
* Core 1 continuously reads the samples array and outputs the values to the I2S and PWM streams
* according to the pico internal clock (output frequency 48KHz)
* The samples array is continually filled by Core 0, synchronised to the incoming Ultranet stream
* This avoids audio sample sync issues as when the output sequence is running ahead, the Core 1 processing
* simply outputs the previous sample a second time, as it's still in the array.
* Or it outputs the new sample, missing one, if it's running behind.
* Provided the pico clock is at a close enough frequency to the incoming Ultranet stream, these corrections
* are infrequent and the samples very similar in amplitude, so there's no audible click or pop.
*
* An optional selector switch on 3 pins determines which channels will be presented on which I2S or PWM outputs.
* The most significant switch pin selects the Ultranet stream (1-8 or 9-16), whereas the LS two pins select
* which pair of channels (1-2, 3-4, 5-6 or 7-8) appear on a particular I2S and PWM output pair.
* Switch is read once at boot time, so reset required when switch position changed.
*
* [한국어]
* 코어1 은 samples 배열을 계속 읽어, Pico 내부 클럭(출력 주파수 48kHz)에 맞춰 I2S 와 PWM 으로 출력한다.
* samples 배열은 코어0 이 Ultranet 입력 스트림에 동기되어 계속 채운다.
* 출력이 입력보다 빠르면 코어1 은 배열에 남아 있는 이전 샘플을 한 번 더 출력하고,
* 느리면 새 샘플을 출력하면서 하나를 건너뛴다. 이렇게 해서 두 클럭 간 동기 문제를 피한다.
* Pico 클럭이 Ultranet 클럭과 충분히 가까우면 이런 보정은 드물게 일어나고,
* 인접 샘플의 크기가 거의 같으므로 귀에 들리는 클릭/팝 노이즈는 생기지 않는다.
*
* 3핀 셀렉터 스위치(선택 사항)로 어떤 채널이 어느 I2S/PWM 출력에 나올지 정한다.
* 최상위 비트는 Ultranet 스트림(1-8 또는 9-16)을, 하위 두 비트는 특정 I2S/PWM 출력 쌍에
* 나올 채널 쌍(1-2, 3-4, 5-6, 7-8)을 선택한다.
* 스위치는 부팅 시 한 번만 읽으므로, 스위치 위치를 바꾸면 리셋해야 한다.
*/

#include "ultranet.h"

volatile uint8_t slice[4];                                  // PWM slice numbers for specified pins   (지정 핀에 해당하는 PWM 슬라이스 번호)

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

// I2S 출력용 PIO 상태 머신 하나를 설정한다 (아직 시작하지 않음 - core1_entry 에서 4개를 동시에 시작)
// 핀 배치: pin = DATA, pin+1 = BCLK, pin+2 = LRCLK
void i2s_pio_init(PIO pio, uint sm, uint pin, uint offset)
{
    pio_sm_config c = i2s_program_get_default_config(offset);  // get default structure             (기본 설정 구조체)
    pio_gpio_init(pio, pin);
    pio_gpio_init(pio, pin+1);
    pio_gpio_init(pio, pin+2);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 3, true);  // set base+3 pins to output              (기준 핀부터 3개를 출력으로)
    sm_config_set_clkdiv_int_frac(&c, AUDIV, 0);            // set frequency of UNET_SM to fs x 256   (PIO 클럭 분주 설정 - 실제로는 I2S 상태 머신)
    sm_config_set_out_pins (&c, pin, 3);                    // out pin range base and count           (out 명령용 핀 기준과 개수)
    sm_config_set_sideset_pins (&c, pin+1);                 // sideset pin range base                 (사이드셋 핀 기준 = BCLK)
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // configure 8 depth output fifo          (TX FIFO 8단)
    sm_config_set_out_shift(&c, false, false, 32);          // set shift left, no autpull for out FIFO (왼쪽 시프트(MSB 먼저), 자동 pull 없음)
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine       (상태 머신에 설정 적용)
}

// PWM 슬라이스의 A(왼쪽)/B(오른쪽) 채널 듀티 값을 설정하는 매크로
#define pwm_set_a(slice,num) pwm_set_chan_level((slice), PWM_CHAN_A, (uint16_t)(num))
#define pwm_set_b(slice,num) pwm_set_chan_level((slice), PWM_CHAN_B, (uint16_t)(num))

// PWM 오디오 출력 8개(슬라이스 4개 x A/B)를 설정하고 시작한다
// 주기 4096(12비트) → PWM 주파수 = 196.5MHz / 4096 ≒ 48kHz
void pwm_setup(void)
{
    uint count;                                             // loop counter                            (루프 카운터)
    gpio_set_function(PIN_PWM_1A, GPIO_FUNC_PWM);           // set pin funtion to PWM output           (핀 기능을 PWM 출력으로)
    gpio_set_function(PIN_PWM_1B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_2A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_2B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_3A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_3B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_4A, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    gpio_set_function(PIN_PWM_4B, GPIO_FUNC_PWM);           // set pin funtion to PWM output
    slice[0] = pwm_gpio_to_slice_num(PIN_PWM_1A);           // get PWM slice that uses this pin        (이 핀을 쓰는 PWM 슬라이스 번호)
    slice[1] = pwm_gpio_to_slice_num(PIN_PWM_2A);           // get PWM slice that uses this pin
    slice[2] = pwm_gpio_to_slice_num(PIN_PWM_3A);           // get PWM slice that uses this pin
    slice[3] = pwm_gpio_to_slice_num(PIN_PWM_4A);           // get PWM slice that uses this pin
    for(count=0;count<4;count++)
    {
        pwm_set_wrap(slice[count], 0xFFF);                  // set PWM period to 4096                  (PWM 주기 4096)
        pwm_set_a(slice[count], 0x1000/2);                  // start output at 50%                     (듀티 50% = 무음에서 시작)
        pwm_set_b(slice[count], 0x1000/2);                  // start output at 50%
        pwm_set_enabled(slice[count], true);                // start PWM running                       (PWM 시작)
    }
}
// 코어1 이 사용하는 핀(I2S, 셀렉터 스위치, PWM)을 picotool 용 바이너리 정보로 등록한다
void set_core1_info(void)
{
    bi_decl(bi_pin_mask_with_name((1<<I2S1_PINS|(1<<I2S1_PINS+1)|(1<<I2S1_PINS+2)), "I2S_1 DATA,BCLK,LRCLK"));
    bi_decl(bi_pin_mask_with_name((1<<I2S2_PINS|(1<<I2S2_PINS+1)|(1<<I2S2_PINS+2)), "I2S_2"));
    bi_decl(bi_pin_mask_with_name((1<<I2S3_PINS|(1<<I2S3_PINS+1)|(1<<I2S3_PINS+2)), "I2S_3"));
    bi_decl(bi_pin_mask_with_name((1<<I2S4_PINS|(1<<I2S4_PINS+1)|(1<<I2S4_PINS+2)), "I2S_4"));
    bi_decl(bi_pin_mask_with_name((1<<SELECTOR_SW_BASE|(1<<SELECTOR_SW_BASE+1)|(1<<SELECTOR_SW_BASE+2)), "SELECTOR_SWITCH"));
    bi_decl(bi_4pins_with_names(PIN_PWM_1A, "PWM_1 Left", PIN_PWM_1B, "PWM_1 Right", PIN_PWM_2A, "PWM_2 Left", PIN_PWM_2B, "PWM_2 Right"));
    bi_decl(bi_4pins_with_names(PIN_PWM_3A, "PWM_3 Left", PIN_PWM_3B, "PWM_3 Right", PIN_PWM_4A, "PWM_4 Left", PIN_PWM_4B, "PWM_4 Right"));
}


// 코어1 진입점: 출력 하드웨어를 초기화한 뒤, samples[] 를 I2S/PWM 으로 끝없이 출력한다
void core1_entry(void)                                      // Core1 starts executiing here
{
    uint i2s_offset;                                        // position for i2s code in pio (shared for all SMs)   (PIO 안의 I2S 코드 위치, 모든 SM 공유)
    volatile uint32_t ssample;                              // signed version of audio sample                     (PWM 용으로 변환한 샘플 - 실제로는 부호 없는 오프셋 바이너리)
    uint selector = get_selector();                         // read selector switch                               (셀렉터 스위치 읽기 - 현재 사용되지 않음)
    int count;                                              // general purpose counter                            (범용 카운터)
    uint ch[8], sel_sw;                                     // channel index into samples array from selector switch   (스위치로 정한 samples 배열 인덱스)

    sel_sw = get_selector();                                // read selector switch GPIO pins                     (셀렉터 스위치 GPIO 읽기)
    // 출력 순번 count 에 스위치 하위 2비트 x 2 만큼 오프셋을 더해 채널을 회전시킨다.
    // (sel_sw<<1) 에서 bit2(스트림 선택)는 8 이 되어 & 7 로 사라지므로 채널 계산에는 하위 2비트만 영향을 준다.
    for(count=0;count<8;count++)
    {
        ch[count] = (count + (sel_sw<<1)) & 7;              // offset channel number by selector switch setting   (스위치 설정만큼 채널 번호 오프셋)
    }                                                       // (Lower two switch bits determine channel selection)

    pwm_setup();                                            // initialise PWM hardware and start outputs          (PWM 초기화 및 출력 시작)

#ifdef MCLK
    mclk_pio_init(MCLK_PIO, MCLK_SM, MCLK_PIN);             // uncomment to enable I2S MCLK                       (I2S MCLK 출력 시작)
#endif // MCLK

    i2s_offset = pio_add_program(I2S_PIO, &i2s_program);    // load i2c output code once for all state machines   (I2S 프로그램을 한 번만 적재)
    i2s_pio_init(I2S_PIO, 0, I2S1_PINS, i2s_offset);        // all 4 state machines use the same code             (4개 상태 머신이 같은 코드 사용)
    i2s_pio_init(I2S_PIO, 1, I2S2_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 2, I2S3_PINS, i2s_offset);
    i2s_pio_init(I2S_PIO, 3, I2S4_PINS, i2s_offset);
    sleep_ms(200);                                          // wait for incoming samples to start                 (입력 샘플이 들어오기 시작할 때까지 대기)
    // ensure there is data in each output FIFO before starting the state machines
    // 상태 머신을 시작하기 전에 각 출력 FIFO 에 데이터를 미리 넣어 둔다 (왼쪽, 오른쪽 순)
    pio_sm_put_blocking(I2S_PIO, 0, samples[ch[0]]);        // subframe 1 goes to I2S0 channel 1   (서브프레임 1 → I2S0 왼쪽)
    pio_sm_put_blocking(I2S_PIO, 1, samples[ch[2]]);        // subframe 3 goes to I2S1 channel 1   (서브프레임 3 → I2S1 왼쪽)
    pio_sm_put_blocking(I2S_PIO, 2, samples[ch[4]]);        // subframe 5 goes to I2S2 channel 1   (서브프레임 5 → I2S2 왼쪽)
    pio_sm_put_blocking(I2S_PIO, 3, samples[ch[6]]);        // subframe 7 goes to I2S3 channel 1   (서브프레임 7 → I2S3 왼쪽)
    pio_sm_put_blocking(I2S_PIO, 0, samples[ch[1]]);        // subframe 2 goes to I2S0 channel 2   (서브프레임 2 → I2S0 오른쪽)
    pio_sm_put_blocking(I2S_PIO, 1, samples[ch[3]]);        // subframe 4 goes to I2S1 channel 2   (서브프레임 4 → I2S1 오른쪽)
    pio_sm_put_blocking(I2S_PIO, 2, samples[ch[5]]);        // subframe 6 goes to I2S2 channel 2   (서브프레임 6 → I2S2 오른쪽)
    pio_sm_put_blocking(I2S_PIO, 3, samples[ch[7]]);        // subframe 8 goes to I2S3 channel 2   (서브프레임 8 → I2S3 오른쪽)

    pio_set_sm_mask_enabled(I2S_PIO, 0xF, true);            // enable all I2S state machines at the same instant   (I2S 상태 머신 4개를 동시에 시작)

    // 메인 출력 루프. pio_sm_put_blocking 은 FIFO 에 빈 자리가 생길 때까지 기다리므로
    // 루프 속도는 I2S 출력 클럭(48kHz)에 자동으로 맞춰진다.
    while(true)                                             // output samples synchronised with I2S streams
    {
        // continually load pio FIFOs for I2S outputs, and PWM registers for PWM outputs
        // I2S 출력용 PIO FIFO 와 PWM 출력용 레지스터를 계속 채운다.
        // PWM 값: 부호 있는 샘플에 0x80000000 을 더해 오프셋 바이너리로 바꾼 뒤 상위 12비트(>>20)만 사용한다.
        pio_sm_put_blocking(I2S_PIO, 0, samples[ch[0]]);    // subframe 1 goes to I2S0 channel 1   (서브프레임 1 → I2S0 왼쪽)

        ssample = (0x80000000 + (signed)samples[ch[0]]);
        pwm_set_a(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio  (PWM 값 = 오디오 상위 12비트)

        pio_sm_put_blocking(I2S_PIO, 1, samples[ch[2]]);    // subframe 3 goes to I2S1 channel 1

        ssample = (0x80000000 + (signed)samples[ch[2]]);
        pwm_set_a(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[ch[4]]);    // subframe 5 goes to I2S2 channel 1

        ssample = (0x80000000 + (signed)samples[ch[4]]);
        pwm_set_a(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[ch[6]]);    // subframe 7 goes to I2S3 channel 1

        ssample = (0x80000000 + (signed)samples[ch[6]]);
        pwm_set_a(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 0, samples[ch[1]]);    // subframe 2 goes to I2S0 channel 2   (서브프레임 2 → I2S0 오른쪽)

        ssample = (0x80000000 + (signed)samples[ch[1]]);
        pwm_set_b(slice[0], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 1, samples[ch[3]]);    // subframe 4 goes to I2S1 channel 2

        ssample = (0x80000000 + (signed)samples[ch[3]]);
        pwm_set_b(slice[1], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 2, samples[ch[5]]);    // subframe 6 goes to I2S2 channel 2

        ssample = (0x80000000 + (signed)samples[ch[5]]);
        pwm_set_b(slice[2], (ssample>>20));                 // PWM value is high 12 bits of audio

        pio_sm_put_blocking(I2S_PIO, 3, samples[ch[7]]);    // subframe 8 goes to I2S3 channel 2

        ssample = (0x80000000 + (signed)samples[ch[7]]);
        pwm_set_b(slice[3], (ssample>>20));                 // PWM value is high 12 bits of audio
    }
}
