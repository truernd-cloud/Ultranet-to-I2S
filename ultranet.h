
/*
* Header file for the Ultranet project
* contains all #includes and #defines for the
* project C files
*
* Ultranet provides 16 separate audio channels (or 8 stereo pairs) via two twisted pairs in a
* CAT5/CAT6 cable. Each pair carries 8 channels, and is termed a "stream" in this project.
* This module decodes a single Ultranet stream, but can select between both available streams.
*
* The module can be used either as an all-channels decoder (8 simultaneous channels) or as a
* specific decoder for selected channels. An optional binary selector switch sets the offset for
* the order of output channels onto board pins, such that a single output pair (I2S or PWM) can
* be selected to be any of the 8 available stereo pairs from the two Ultranet input streams.
*
* [한국어]
* Ultranet 프로젝트 공용 헤더 파일.
* 프로젝트의 모든 C 파일이 사용하는 #include 와 #define 을 모아 둔다.
*
* Ultranet 은 CAT5/CAT6 케이블 안의 트위스트 페어 두 쌍으로 16개의 독립 오디오 채널
* (스테레오 8쌍)을 전송한다. 한 쌍이 8채널을 실어 나르며, 이 프로젝트에서는 이를
* "스트림(stream)" 이라고 부른다. 이 모듈은 한 번에 스트림 하나만 디코딩하지만,
* 두 스트림 중 어느 것을 받을지 선택할 수 있다.
*
* 모듈은 전 채널 디코더(8채널 동시 출력)로 쓰거나, 특정 채널만 뽑아내는 디코더로 쓸 수 있다.
* 선택 사항인 3비트 이진 셀렉터 스위치로 출력 채널이 보드 핀에 배치되는 순서(오프셋)를
* 바꿀 수 있으므로, 출력 한 쌍(I2S 또는 PWM)만 사용하더라도 두 Ultranet 입력 스트림의
* 스테레오 8쌍 중 원하는 쌍을 골라 낼 수 있다.
*/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"

// "ultranet.pio" 를 pioasm 으로 변환해 빌드 시 자동 생성되는 헤더
// (주의: 빌드 디렉터리 이름이 "build" 이고 소스 폴더 바로 아래에 있어야 찾을 수 있다)
#include "build/ultranet.pio.h"     // derived automatically from the "ultranet.pio" source file

// strings for inclusion in binary info (for query by picotool)
// picotool 로 조회할 수 있도록 바이너리에 삽입되는 설명/버전 문자열
#define DESCRIPTION "Single Ultranet stream input (sw selected), 4xI2S stereo, 8xPWM mono"
#define VERSION "1.2"

// conditional compilation switches for hardware options
// 하드웨어 옵션용 조건부 컴파일 스위치
// #define DEBUG                    // enable debug code DEBUG DEBUG DEBUG
                                    // (디버그 코드 활성화 - USB 시리얼로 LED 색상 테스트, 디코딩은 하지 않음)
#define WS2812                      // Our board has a ws2812 programmable LED
                                    // (보드에 WS2812 RGB LED 가 있음)
#define MCLK                        // Enable MCLK clock for I2S devices
                                    // (I2S 장치용 마스터 클럭 MCLK 출력 사용)

// 시스템 클럭(kHz). PIO 가 Ultranet 바이페이즈 클럭(24.576MHz)의 7배 또는 8배로 샘플링하도록 설정한다.
// CLOCKSPEED 와 AUDIV, 그리고 ultranet.pio 의 "cy" 값은 반드시 함께 맞춰서 바꿔야 한다.
                                    // 172000 for 7 slots per bit incoming Ultranet stream
#define CLOCKSPEED  196500          // 196500 for 8 slots per bit incoming Ultranet stream
#define AUDIV 8                     // Audio divider for pio timing (7 for 172MHz, 8 for 196.5MHz)
                                    // (오디오 PIO 분주비: 196.5MHz / 8 ≒ 24.56MHz ≒ 512 x 48kHz, 즉 MCLK(256fs)의 2배)
// Ultranet input and MCLK state machines use pio0
// Ultranet 입력과 MCLK 상태 머신은 pio0 을 사용
#define UNETL_PIN 0                 // ultranet low stream (1-8) input pin      (하위 스트림 1-8 입력 핀)
#define UNETH_PIN 1                 // ultranet high stream (9-16) input pin    (상위 스트림 9-16 입력 핀)
#define UNET_PIN UNETL_PIN          // ultranet default input pin               (기본 입력 핀)
#define UNET_PIO pio0               // PIO module to use for Ultranet input     (Ultranet 입력용 PIO)
#define UNET_SM 0                   // state machine to use for Ultranet input  (Ultranet 입력용 상태 머신)
#ifdef MCLK                         // if we want an I2S MCLK clock
    // 참고: 공식 Raspberry Pi Pico 보드에서는 GP24 가 VBUS 감지용으로 내부 연결되어 있어 핀 헤더로 나오지 않는다
    #define MCLK_PIN 24             // I2S Master Clock Pin (if used)           (I2S 마스터 클럭 핀)
    #define MCLK_PIO pio0           // state machine for I2S master clock      (MCLK 용 PIO)
    #define MCLK_SM 1               // state machine for I2S master clock      (MCLK 용 상태 머신)
#endif // MCLK
// I2S outputs use second pio (pio1), four I2S outputs, 3 pins each
// I2S 출력은 두 번째 PIO(pio1)의 상태 머신 4개를 모두 사용. 출력 4개, 각 3핀(DATA, BCLK, LRCLK)
#define I2S_PIO pio1                // PIO 1 is dedicated to I2S outputs (all 4 SMs)
#define I2S1_PINS 2                 // base for I2S output pins (3 pins starting point)   (I2S1: GP2~4)
#define I2S2_PINS 5                 // base for I2S output pins (3 pins starting point)   (I2S2: GP5~7)
#define I2S3_PINS 8                 // base for I2S output pins (3 pins starting point)   (I2S3: GP8~10)
#define I2S4_PINS 17                // base for I2S output pins (3 pins starting point)   (I2S4: GP17~19)
// Selector binary switch (3 pole)
// 3비트 이진 셀렉터 스위치 (GP11~13). 스위치 공통 단자를 0V 또는 3.3V 중 어디에 연결했는지 하나만 정의한다.
#define SELECTOR_SW_BASE 11         // base pin (switch is 3-pin, base+2) switches to ground
#define SW_COMM_LOW                 // switch common pin(s) are connected to 0v     (공통 단자 = 0V, 내부 풀업 사용)
// #define SW_COMM_HIGH             // switch common pin(s) are connected to 3.3v   (공통 단자 = 3.3V, 내부 풀다운 사용)
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
    #define LED_STREAM_COLOUR BLUE  // set colour for LED stream indication        (스트림 수신 표시 색)
    #define LED_STREAM_MASK 0xFFFF00FF  // Mask blue bits, for stream detect LED   (파란색 비트만 지우는 마스크)
    #define put_pixel(pixel) pio_sm_put(WS2812_PIO, WS2812_SM, (pixel))
#endif // WS2812
// #define PICO_LED 25                 // Uncomment to use normal LED on standard PICO boards
                                       // (일반 Pico 보드의 기본 LED(GP25)를 쓰려면 주석 해제)
#define STREAM_LED_RESET 200000     // Period in us to reset stream indicator LED  (스트림 LED 를 리셋하는 주기, us)
// for PWM analog audio outputs
// PWM 아날로그 오디오 출력 핀 (각 PWM 슬라이스의 A = 왼쪽, B = 오른쪽). 출력에 RC 저역 통과 필터가 필요하다.
// 참고: 공식 Pico 보드에서 GP29 는 VSYS 전압 측정용으로 내부 연결되어 있어 사용할 수 없다.
#define PIN_PWM_1A 14               // A channel of PWM slice (left audio)
#define PIN_PWM_1B 15               // B channel of PWM slice (right audio)
#define PIN_PWM_2A 20               // A channel of PWM slice (left audio)
#define PIN_PWM_2B 21               // B channel of PWM slice (right audio)
#define PIN_PWM_3A 26               // A channel of PWM slice (left audio)
#define PIN_PWM_3B 27               // B channel of PWM slice (right audio)
#define PIN_PWM_4A 28               // A channel of PWM slice (left audio)
#define PIN_PWM_4B 29               // B channel of PWM slice (right audio)

// these need to be "volatile" otherwise the compiler optimises them out!
// 두 코어(및 인터럽트)가 공유하는 변수이므로 반드시 volatile 이어야 컴파일러가 최적화로 없애지 않는다
extern volatile uint32_t samples[8]; // array of samples read from Ultranet stream   (Ultranet 에서 읽은 8채널 샘플 버퍼)
extern volatile uint32_t led_state; // current value last sent to WS2812 LED         (WS2812 LED 에 마지막으로 보낸 값)

extern void core1_entry(void);      // main process for core 1, defined in "core1.c"  (코어1 메인 함수)
extern void set_core1_info(void);   // set binary info for pins used by core1         (코어1 사용 핀의 바이너리 정보)
extern uint get_selector(void);     // return selector switch state in low 3 bits     (셀렉터 스위치 상태를 하위 3비트로 반환)
