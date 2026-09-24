
/*
* Header file for the Ultranet project
* contains all #includes and #defines for the
* project C files
*
* Ultranet provides 16 separate audio channels (or 8 stereo pairs) via two twisted pairs in a
* CAT5/CAT6 cable. Each pair carries 8 channels, and is termed a "stream" in this project.
* This module decodes both Ultranet streams at the same time (16 channels), mixes all 16
* channels down to a single stereo pair, and outputs the mix on one I2S output.
* The level and pan of each channel in the mix are set by the mix table in "core1.c",
* and can be changed at run time by MIDI Control Change messages (UART MIDI input).
*
* [한국어]
* Ultranet 프로젝트 공용 헤더 파일.
* 프로젝트의 모든 C 파일이 사용하는 #include 와 #define 을 모아 둔다.
*
* Ultranet 은 CAT5/CAT6 케이블 안의 트위스트 페어 두 쌍으로 16개의 독립 오디오 채널
* (스테레오 8쌍)을 전송한다. 한 쌍이 8채널을 실어 나르며, 이 프로젝트에서는 이를
* "스트림(stream)" 이라고 부른다. 이 모듈은 두 스트림(16채널)을 동시에 디코딩하고,
* 16채널을 스테레오 2채널로 믹스해 I2S 출력 하나로 내보낸다.
* 채널별 믹스 레벨과 팬은 "core1.c" 의 믹스 테이블에서 설정하며,
* 동작 중에는 MIDI 컨트롤 체인지 메시지(UART MIDI 입력)로 바꿀 수 있다.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

// "ultranet.pio" 를 pioasm 으로 변환해 빌드 시 자동 생성되는 헤더
// (주의: 빌드 디렉터리 이름이 "build" 이고 소스 폴더 바로 아래에 있어야 찾을 수 있다)
#include "build/ultranet.pio.h"     // derived automatically from the "ultranet.pio" source file

// strings for inclusion in binary info (for query by picotool)
// picotool 로 조회할 수 있도록 바이너리에 삽입되는 설명/버전 문자열
#define DESCRIPTION "Dual Ultranet stream input (16ch), mixed to 1xI2S stereo, MIDI mix control"
#define VERSION "2.1"

// conditional compilation switches for hardware options
// 하드웨어 옵션용 조건부 컴파일 스위치
// #define DEBUG                    // enable debug code DEBUG DEBUG DEBUG
                                    // (디버그 코드 활성화 - USB 시리얼로 LED 색상 테스트, 디코딩은 하지 않음)
#define WS2812                      // Our board has a ws2812 programmable LED
                                    // (보드에 WS2812 RGB LED 가 있음)
#define MCLK                        // Enable MCLK clock for I2S devices
                                    // (I2S 장치용 마스터 클럭 MCLK 출력 사용)
#define MIDI                        // Enable MIDI input (UART) for mix control
                                    // (믹스 조작용 MIDI 입력(UART) 사용)

// 시스템 클럭(kHz). PIO 가 Ultranet 바이페이즈 클럭(24.576MHz)의 7배 또는 8배로 샘플링하도록 설정한다.
// CLOCKSPEED 와 AUDIV, 그리고 ultranet.pio 의 "cy" 값은 반드시 함께 맞춰서 바꿔야 한다.
                                    // 172000 for 7 slots per bit incoming Ultranet stream
#define CLOCKSPEED  196500          // 196500 for 8 slots per bit incoming Ultranet stream
#define AUDIV 8                     // Audio divider for pio timing (7 for 172MHz, 8 for 196.5MHz)
                                    // (오디오 PIO 분주비: 196.5MHz / 8 ≒ 24.56MHz ≒ 512 x 48kHz, 즉 MCLK(256fs)의 2배)
// Ultranet input and MCLK state machines use pio0
// Ultranet 입력 2개와 MCLK 상태 머신은 pio0 을 사용 (SM0, SM1 = Ultranet, SM2 = WS2812, SM3 = MCLK)
#define UNET_PIO pio0               // PIO module to use for Ultranet input     (Ultranet 입력용 PIO)
#define UNETL_PIN 0                 // ultranet low stream (1-8) input pin      (하위 스트림 1-8 입력 핀)
#define UNETL_SM 0                  // state machine for low stream             (하위 스트림용 상태 머신)
#define UNETH_PIN 1                 // ultranet high stream (9-16) input pin    (상위 스트림 9-16 입력 핀)
#define UNETH_SM 1                  // state machine for high stream            (상위 스트림용 상태 머신)
#define UNET_STREAMS 2              // number of streams decoded                (디코딩하는 스트림 수)
#define UNET_CHANNELS (8*UNET_STREAMS)  // total channels decoded               (디코딩하는 전체 채널 수)
#ifdef MCLK                         // if we want an I2S MCLK clock
    // 참고: 공식 Raspberry Pi Pico 보드에서는 GP24 가 VBUS 감지용으로 내부 연결되어 있어 핀 헤더로 나오지 않는다
    #define MCLK_PIN 24             // I2S Master Clock Pin (if used)           (I2S 마스터 클럭 핀)
    #define MCLK_PIO pio0           // state machine for I2S master clock      (MCLK 용 PIO)
    #define MCLK_SM 3               // state machine for I2S master clock      (MCLK 용 상태 머신)
#endif // MCLK
// Single I2S stereo output on pio1
// I2S 스테레오 출력 1개 (pio1 SM0), 3핀(DATA, BCLK, LRCLK)
#define I2S_PIO pio1                // PIO for I2S output                       (I2S 출력용 PIO)
#define I2S_SM 0                    // state machine for I2S output             (I2S 출력용 상태 머신)
#define I2S_PINS 2                  // base for I2S output pins (3 pins starting point)   (I2S: GP2~4)
// MIDI input: UART receive only, 31250 baud. Needs an opto-isolated MIDI IN circuit (e.g. 6N138 / H11L1)
// MIDI 입력: UART 수신 전용, 31250 baud. 광절연 MIDI IN 회로(예: 6N138 / H11L1)가 필요하다
#ifdef MIDI
    #define MIDI_UART uart1         // UART used for MIDI input                   (MIDI 입력용 UART)
    #define MIDI_RX_PIN 5           // UART1 RX pin                               (UART1 RX 핀: GP5)
    #define MIDI_BAUD 31250         // MIDI baud rate                             (MIDI 통신 속도)
#endif // MIDI
// ws2812 multicolour LED driving
// WS2812 컬러 LED 구동 설정
#ifdef WS2812
    #define WS2812_PIN 16           // chinese pico boards have ws2812 on pin 16   (중국산 호환 보드는 GP16 에 WS2812 가 있음)
    #define WS2812_PIO pio0         // same pio as ultranet & mclk                 (Ultranet/MCLK 과 같은 PIO 사용)
    #define WS2812_SM 2             // state machine for LED output                (LED 출력용 상태 머신)
    // WS2812 로 보내는 32비트 값은 상위 24비트가 G-R-B 순서이다
    #define GREEN 0x0F000000        // send this to ws2812 for GREEN LED
    #define RED   0x00180000        // send this to ws2812 for RED LED
    #define BLUE  0x00001F00        // send this to ws2812 for BLUE LED
    #define WHITE 0x0F0F0F00        // send this to ws2812 for WHITE LED
    // 주의: 매크로 이름과 실제 조합 색이 맞지 않는다 (RED|BLUE 는 실제로 마젠타, GREEN|BLUE 는 시안).
    #define CYAN (RED|BLUE)         // above primary colour intensities are tuned for
    #define MAGENTA (GREEN|BLUE)    // best colour mix and even-ness
    #define YELLOW (GREEN|RED)
    #define BLACK 0                 // turn off all LEDs in module                 (모든 LED 끄기)
    #define LED_ERR_COLOUR RED      // set colour for LED frame error indiication  (프레임 오류 표시 색)
    #define LED_STREAML_COLOUR BLUE // set colour for low stream (1-8) indication  (하위 스트림 수신 표시 색)
    #define LED_STREAMH_COLOUR GREEN // set colour for high stream (9-16) indication (상위 스트림 수신 표시 색)
    // Mask stream colour bits, for stream detect LED  (스트림 표시 색 비트만 지우는 마스크)
    #define LED_STREAM_MASK (~(LED_STREAML_COLOUR|LED_STREAMH_COLOUR))
    #define put_pixel(pixel) pio_sm_put(WS2812_PIO, WS2812_SM, (pixel))
#endif // WS2812
// #define PICO_LED 25                 // Uncomment to use normal LED on standard PICO boards
                                       // (일반 Pico 보드의 기본 LED(GP25)를 쓰려면 주석 해제)
#define STREAM_LED_RESET 200000     // Period in us to reset stream indicator LED  (스트림 LED 를 리셋하는 주기, us)

// these need to be "volatile" otherwise the compiler optimises them out!
// 두 코어(및 인터럽트)가 공유하는 변수이므로 반드시 volatile 이어야 컴파일러가 최적화로 없애지 않는다
// samples[0..7] = 하위 스트림 채널 1-8, samples[8..15] = 상위 스트림 채널 9-16
extern volatile uint32_t samples[UNET_CHANNELS]; // array of samples read from Ultranet streams   (Ultranet 에서 읽은 16채널 샘플 버퍼)
extern volatile uint32_t led_state; // current value last sent to WS2812 LED         (WS2812 LED 에 마지막으로 보낸 값)

extern void core1_entry(void);      // main process for core 1, defined in "core1.c"  (코어1 메인 함수)
extern void set_core1_info(void);   // set binary info for pins used by core1         (코어1 사용 핀의 바이너리 정보)
