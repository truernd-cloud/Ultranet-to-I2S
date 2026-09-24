/*
* Behringer Ultranet decoder
* Reads both Ultranet streams, decoding 16 x audio channels
* core1 mixes the 16 channels to one I2S stereo output
*
* Ultranet bit rate = 12.288MHz (8x32bit samples at 48khz)
* Ultranet biphase clock rate = 2 x 12.288MHz = 24.576MHz
* We need pio module to sample at 7 x incoming clock rate
* Ideal system clock therefore = 7 x 24.576 = 172.032MHz
* We set cpu clock frequency as close to 172032KHz as possible
* but can only set certain numbers of MHz, so 172000khz
*
* OR we need pio module to sample at 8 x incoming clock rate
* Ideal system clock therefore = 8 x 24.576 = 196.608MHz
* We set cpu clock frequency as close to 196608KHz as possible
* but can only set certain numbers of MHz, so 196500khz
*
* Ultranet audio sample depth is in fact 22 bits, not 24
* So we mask the 2 LSBs when reading words from Ultranet stream
*
* Use LED to indicate if we have a framing error
* ie we get out of sync with the 8 subframes of the ultranet stream
* if we don't detect start of frame sync in the right place,
* turn on the error LED
*
* Use multicore to compensate for difference in speed between
* Ultranet input stream (clocked from source) and I2S output
* stream (clocked from this pico). Array of samples is filled
* from the Ulranet streams by core0, then read out to I2S using
* core1, asynchronously from core0.
*
* [한국어]
* Behringer Ultranet 디코더
* Ultranet 스트림 두 개를 모두 읽어 오디오 16채널을 디코딩하고,
* 코어1 이 16채널을 믹스해 I2S 스테레오 출력 하나로 내보낸다.
*
* Ultranet 비트 레이트 = 12.288MHz (32비트 샘플 8개 x 48kHz)
* Ultranet 바이페이즈 클럭 = 2 x 12.288MHz = 24.576MHz
* PIO 가 입력 클럭의 7배로 샘플링하려면 이상적인 시스템 클럭은 7 x 24.576 = 172.032MHz.
* 실제로는 설정 가능한 값 중 가장 가까운 172000kHz 를 사용한다.
*
* 또는 입력 클럭의 8배로 샘플링하려면 이상적인 시스템 클럭은 8 x 24.576 = 196.608MHz.
* 설정 가능한 값 중 가장 가까운 196500kHz 를 사용한다. (현재 기본값)
*
* Ultranet 오디오 샘플은 실제로 24비트가 아니라 22비트이므로,
* 스트림에서 읽은 워드의 하위 비트를 마스크한다.
*
* LED 로 프레임 오류(8개 서브프레임과의 동기가 어긋남)를 표시한다.
* 프레임 시작 동기 패턴이 예상 위치에서 검출되지 않으면 오류 LED 를 켠다.
*
* 멀티코어를 이용해 Ultranet 입력(송신 장비 클럭 기준)과 I2S 출력(이 Pico 클럭 기준)의
* 속도 차이를 흡수한다. 코어0 은 Ultranet 스트림 두 개에서 샘플 배열을 채우고,
* 코어1 은 코어0 과 비동기로 그 배열을 읽어 믹스한 뒤 I2S 로 내보낸다.
*/

#include "ultranet.h"

volatile uint32_t samples[UNET_CHANNELS];   // array of samples read from Ultranet streams   (Ultranet 에서 읽은 샘플 배열, 코어1 과 공유)

// Receive state for one Ultranet stream
// Ultranet 스트림 하나의 수신 상태
typedef struct
{
    uint sm;                    // state machine receiving this stream          (이 스트림을 받는 상태 머신)
    uint base;                  // index of first channel in samples[]          (samples[] 안의 첫 채널 위치)
    uint32_t led_colour;        // LED colour to show while stream is received  (수신 중 표시할 LED 색)
    int discard;                // words still to discard after startup         (시작 직후 버릴 남은 워드 수)
    int subframe;               // next subframe expected, -1 = not yet synced  (다음에 올 서브프레임 번호, -1 = 아직 동기 전)
} unet_stream_t;

#ifdef PICO_LED
// Pico 기본 LED 핀을 초기화한다
void ultranet_gpio_init(void)
{
    gpio_init(PICO_LED);                                    // set LED pin as GPIO      (LED 핀을 GPIO 로 설정)
    gpio_set_dir(PICO_LED, GPIO_OUT);                       // set LED pin as output    (LED 핀을 출력으로 설정)
}
#endif // PICO_LED

// state machine init functions (used to be defined in <prog>.pio file)
// 상태 머신 초기화 함수 (예전에는 <prog>.pio 파일 안에 정의되어 있었음)
// Ultranet 입력용 PIO 상태 머신을 설정하고 시작한다. 프로그램은 미리 적재한 offset 을 공유한다.
void ultranet_pio_init(PIO pio, uint sm, uint pin, uint offset)
{
    gpio_set_dir(pin, false);                               // set ultranet pin as input            (입력으로 설정)
    gpio_set_pulls(pin, true, false);                       // set pullup on ultranet pin           (풀업 설정)
    pio_sm_config c = ultranet_program_get_default_config(offset);  // get default structure        (기본 설정 구조체)
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);          // configure 8 depth input fifo         (RX FIFO 를 8단으로 결합)
    sm_config_set_in_pins (&c, pin);                        // input pin range base                 (입력 핀 기준)
    sm_config_set_jmp_pin(&c, pin);                         // specify pin for jmp instructions     (jmp pin 명령이 볼 핀 지정)
    pio_sm_init(pio, sm, offset, &c);                       // apply structure to state machine     (상태 머신에 설정 적용)
    pio_sm_set_enabled(pio, sm, true);                      // start state machine running          (상태 머신 시작)
}


#ifdef WS2812
volatile uint32_t led_state;                                // current value last sent to WS2812 LED   (WS2812 에 마지막으로 보낸 값)
// WS2812 LED 구동용 PIO 상태 머신을 설정한다
void ws2812_pio_init(PIO pio, uint sm, uint pin)            // Set up PIO SM for ws2812 LED module
{
    uint offset = pio_add_program(pio, &ws2812_program);    // PIO program shares code space with UNET and MCLK   (UNET/MCLK 과 코드 공간 공유)
    pio_gpio_init(pio, pin);                                // Set up GPIO pin for PIO...                          (핀을 PIO 에 연결...)
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);  // ...output                                           (...출력으로)

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    // 실제 인자는 shift_right=false(왼쪽 시프트, MSB 먼저), autopull=true, 24비트 단위이다 (아래 영문 주석은 부정확)
    sm_config_set_out_shift(&c, false, true, 24);           // set shift direction RIGHT, no autopull
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);          // use 8 deep TX FIFO                                  (TX FIFO 8단)

#define CYCLES_PER_BIT ((ws2812_T1)+(ws2812_T2)+(ws2812_T3)) // constants defined in .pio source file   (.pio 파일에 정의된 상수)
    // WS2812 는 정확한 800kHz 비트 타이밍이 필요하므로 현재 시스템 클럭 기준으로 분주비를 계산한다
    float div = clock_get_hz(clk_sys) / (800000 * CYCLES_PER_BIT);  // ws2812 needs precise 800KHz timing
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);                      // Set ws2812 PIO state machine running    (상태 머신 시작)
    led_state = BLACK;                                      // initialise WS2812 LED to all off        (LED 상태를 꺼짐으로 초기화)
    pio_sm_put(pio, sm, led_state);                         // Clear all LED colours to off            (LED 끄기)
}
#endif // WS2812

/*
* Timer callback for periodically turning off Ultranet detected LED
*  Outputs current state (as turned on by Ultranet stream code) then clears flag
* If no Ultranet stream received, LED will turn off at next alarm tick
*
* [한국어]
* Ultranet 수신 표시 LED 를 주기적으로 끄기 위한 타이머 콜백.
* 현재 상태(Ultranet 디코딩 코드가 켠 값)를 LED 로 출력한 뒤 스트림 표시 비트를 지운다.
* 다음 주기까지 Ultranet 스트림이 들어오지 않으면 다음 알람에서 LED 가 꺼진다.
* 오류 표시(빨강) 비트는 지우지 않으므로 한 번 켜지면 재부팅 전까지 유지된다.
*/
int64_t alarm_callback(alarm_id_t id, __unused void *repeatptr)
{
#ifdef WS2812
    put_pixel(led_state);                                   // Output current sate of led_state flag to LED   (현재 led_state 를 LED 로 출력)
#endif // WS2812
#ifdef PICO_LED
    if((led_state & ~LED_STREAM_MASK) > 0)                  // led_state has been set by Ultranet stream code (스트림 코드가 비트를 켰으면)
        gpio_put(PICO_LED, 1);                              // turn on LED when stream is detected            (스트림 검출 시 LED 켜기)
    else
        gpio_put(PICO_LED, 0);                              // or turn off if no stream detected              (아니면 끄기)
#endif // PICO_LED
    led_state = led_state & LED_STREAM_MASK;                // Zero out the stream LED colour bits            (스트림 표시 색 비트만 0 으로)
    // 반환값(양수)은 다음 알람까지의 시간(us). main() 의 repeat_us 를 가리키는 포인터에서 읽는다
    return *(const uint32_t*)repeatptr;                     // return value is repeat time
}

// Embedded binary information (for picotool interrogation of programmed device)
// picotool 로 장치를 조회할 때 보이는 바이너리 정보(설명, 버전, 사용 핀)를 등록한다
void set_binary_info(void)
{
    bi_decl(bi_program_description(DESCRIPTION));           // Description field for embedded identification   (설명)
    bi_decl(bi_program_version_string(VERSION));            // Version field for embedded identification       (버전)
    bi_decl(bi_2pins_with_names(UNETL_PIN, "Ultranet Low (1-8) Stream Input", UNETH_PIN, "Ultranet High (9-16) Input"));
#ifdef MCLK
    bi_decl(bi_1pin_with_name(MCLK_PIN, "I2S MCLK Output"));
#endif // MCLK
#ifdef WS2812
    bi_decl(bi_1pin_with_name(WS2812_PIN, "WS2812 NeoPixel LED"));
#endif // WS2812
#ifdef PICO_LED
    bi_decl(bi_1pin_with_name(PICO_LED, "PICO board normal LED"));
#endif // PICO_LED
    set_core1_info();                                       // info for pins used by core1   (코어1 이 사용하는 핀 정보)
}

/*
* Process one 32 bit word received from an Ultranet stream
* Words are subframes 1-8 of a frame, subframe 1 carries the frame start sync pattern
* (lower 6 bits are 0x0B or 0x0F)
*
* [한국어]
* Ultranet 스트림에서 받은 32비트 워드 하나를 처리한다.
* 워드는 프레임의 서브프레임 1~8 이며, 서브프레임 1 에는 프레임 시작 동기 패턴이 있다
* (하위 6비트가 0x0B 또는 0x0F).
*/
static inline void unet_process_word(unet_stream_t *s, uint32_t word)
{
    bool frame_start = ((word & 0x3F) == 0x0000000B) || ((word & 0x3F) == 0x0000000F);

    // sync with ultranet frames initially, so we don't turn LED on at start
    // 시작 직후에는 워드를 버려 수신을 안정시킨다 (부팅 시 오류 LED 가 켜지지 않도록)
    if(s->discard > 0)
    {
        s->discard--;
        return;
    }

    // 프레임 시작이 와야 할 위치(또는 아직 동기 전)라면 시작 동기 패턴인지 확인한다
    if(s->subframe <= 0)
    {
        if(!frame_start)
        {
            // 동기 후에 시작 프레임이 있어야 할 위치에서 찾지 못함: 오류 색(빨강)을 켠다 (다른 색 비트는 유지)
            // (아직 동기 전이면 오류로 보지 않고 계속 찾는다)
            if(s->subframe == 0)
                led_state = led_state | LED_ERR_COLOUR;     // turn on RED, preserving other colours
            return;
        }
        s->subframe = 0;
    }

    // (word << 4) 로 오디오 데이터를 MSB 쪽으로 정렬하고, 하위 비트(동기/상태 비트)를 마스크한다.
    samples[s->base + s->subframe] = (word << 4) & 0xFFFFFC00;  // move 22 bits of audio into MSBs    (22비트 오디오를 MSB 로 이동)

    if(++s->subframe == 8)
    {
        // 정상 프레임 수신 완료: 이 스트림의 표시 색을 켜고, 다음 워드는 프레임 시작이어야 한다
        s->subframe = 0;
        led_state = led_state | s->led_colour;              // set stream LED on, preserving other colours
    }
}

int main()
{
    const uint64_t repeat_us = STREAM_LED_RESET;            // Repeat time period for alarm to clear Ultranet stream LED   (스트림 LED 리셋 알람 주기)
    uint unet_offset;                                       // position for ultranet code in pio (shared by both SMs)       (PIO 안의 Ultranet 코드 위치, 두 SM 공유)
    // 두 스트림의 수신 상태. 시작 후 처음 200 워드는 버리고, 그 뒤 프레임 시작을 찾아 동기를 맞춘다
    unet_stream_t streams[UNET_STREAMS] = {
        { .sm = UNETL_SM, .base = 0, .led_colour = LED_STREAML_COLOUR, .discard = 200, .subframe = -1 },
        { .sm = UNETH_SM, .base = 8, .led_colour = LED_STREAMH_COLOUR, .discard = 200, .subframe = -1 },
    };

    set_binary_info();                                      // info for querying by picotool                         (picotool 조회용 정보)
    stdio_init_all();                                       // initialise SDK libraries and interfaces               (SDK 표준 입출력 초기화)
    // 시스템 클럭을 오버클럭한다. required=false 이므로 정확히 설정할 수 없으면 실패하며, 반환값은 확인하지 않는다
    set_sys_clock_khz(CLOCKSPEED,false);                    // set cpu clock frequency

#ifdef DEBUG
    sleep_ms(5000);                                         // allow time for USB serial to connect                  (USB 시리얼 연결 대기)
#else
    sleep_ms(500);                                          // allow time for clocks etc. to settle                  (클럭 안정화 대기)
#endif // DEBUG

#ifdef WS2812
    ws2812_pio_init(WS2812_PIO, WS2812_SM, WS2812_PIN);     // ws2812 output pio state machine                       (WS2812 출력 상태 머신)
#endif // WS2812

#ifdef PICO_LED
    ultranet_gpio_init();                                   // initialise required GPIO pins                         (필요한 GPIO 초기화)
#endif // PICO_LED

    // repeat_us 는 main() 의 지역 변수지만 main() 은 끝나지 않으므로 포인터가 계속 유효하다
    add_alarm_in_us(repeat_us, alarm_callback, (void*)&repeat_us, false);  // start timer for stream LED blanking   (스트림 LED 소등 타이머 시작)

    // 두 스트림을 모두 수신한다. 프로그램은 한 번만 적재하고 두 상태 머신이 공유한다
    unet_offset = pio_add_program(UNET_PIO, &ultranet_program);   // load ultranet code once for both state machines  (Ultranet 프로그램 한 번만 적재)
    ultranet_pio_init(UNET_PIO, UNETL_SM, UNETL_PIN, unet_offset); // low stream (1-8)                                (하위 스트림 1-8)
    ultranet_pio_init(UNET_PIO, UNETH_SM, UNETH_PIN, unet_offset); // high stream (9-16)                              (상위 스트림 9-16)

    multicore_launch_core1(core1_entry);                    // start core 1                                  (코어1 시작)

    sleep_ms(100);                                          // wait for core1 to start                       (코어1 시작 대기)
#ifdef DEBUG
    // 디버그 모드: USB 시리얼로 받은 문자에 따라 LED 색을 바꾸는 테스트 루프 (여기서 빠져나오지 않으므로 디코딩은 하지 않음)
    sleep_ms(5000);
    puts("FINISHED setting everything up\n");
#ifdef WS2812
    while(true)
    {
        int inc = getchar();
        switch(inc)
        {
            case 'r':
            case 'R':
                pio_sm_put(WS2812_PIO, WS2812_SM, RED);
                puts("RED");
                break;
            case 'g':
            case 'G':
                pio_sm_put(WS2812_PIO, WS2812_SM, GREEN);
                puts("GREEN");
                break;
            case 'b':
            case 'B':
                pio_sm_put(WS2812_PIO, WS2812_SM, BLUE);
                puts("BLUE");
                break;
            case 'm':
            case 'M':
                pio_sm_put(WS2812_PIO, WS2812_SM, MAGENTA);
                puts("MAGENTA");
                break;
            case 'c':
            case 'C':
                pio_sm_put(WS2812_PIO, WS2812_SM, CYAN);
                puts("CYAN");
                break;
            case 'y':
            case 'Y':
                pio_sm_put(WS2812_PIO, WS2812_SM, YELLOW);
                puts("YELLOW");
                break;
            case 'w':
            case 'W':
                pio_sm_put(WS2812_PIO, WS2812_SM, WHITE);
                puts("WHITE");
                break;
        }
#endif // WS2812
    }
#endif // DEBUG

    // 메인 디코딩 루프: 두 스트림의 RX FIFO 를 번갈아 확인해, 도착한 워드를 처리한다.
    // 스트림마다 워드가 약 2.6us 간격으로 오고 FIFO 가 8단(약 20us)이므로 여유가 충분하다.
    // 한쪽 스트림이 연결되지 않아도 그 FIFO 가 비어 있을 뿐, 다른 스트림 수신에는 영향이 없다.
    while (true)
    {
        for(int n = 0; n < UNET_STREAMS; n++)
        {
            if(!pio_sm_is_rx_fifo_empty(UNET_PIO, streams[n].sm))
                unet_process_word(&streams[n], pio_sm_get(UNET_PIO, streams[n].sm));
        }
    }
}
