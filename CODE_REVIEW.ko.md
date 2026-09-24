# 코드 리뷰 (v1.2)

대상: `ultranet.h`, `ultranet.c`, `core1.c`, `ultranet.pio`, `CMakeLists.txt`

이 리뷰에서는 코드 동작을 바꾸지 않았습니다. 소스 파일에는 한국어 주석만 추가했고, 아래 항목은 수정 제안입니다. 중요도 순으로 정리했습니다.

## 요약

전체 구조는 단순하고 목적에 잘 맞습니다. PIO 로 바이페이즈 신호를 디코딩하고, 두 코어를 입력 클럭 영역과 출력 클럭 영역으로 나눠 클럭 차이를 흡수하는 설계는 적은 코드로 효과를 냅니다. 다만 다음 문제가 있습니다.

- Linux/macOS 에서 **빌드가 바로 실패**합니다 (파일 이름 대소문자).
- 일부 조건부 컴파일 조합(`WS2812` 끄기, `DEBUG` 만 켜기)이 **컴파일되지 않습니다**.
- 시스템 클럭 설정 실패를 확인하지 않아 **아무 표시 없이 오동작**할 수 있습니다.
- 주석 여러 곳이 현재 코드(196.5MHz / 8 분주)와 맞지 않습니다.

## 높음

### 1. `CMakeLists.txt`: 소스 파일 이름 대소문자 불일치

```cmake
add_executable(Ultranet Ultranet.c core1.c ultranet.h)
```

실제 파일은 `ultranet.c` 입니다. Windows 에서는 대소문자를 구분하지 않아 빌드되지만, Linux 와 대소문자 구분 macOS 에서는 `Cannot find source file: Ultranet.c` 로 CMake 단계에서 실패합니다.

**제안:** `Ultranet.c` 를 `ultranet.c` 로 고칩니다.

### 2. `ultranet.h`: 생성 헤더 경로가 빌드 디렉터리 이름에 의존

```c
#include "build/ultranet.pio.h"
```

`pico_generate_pio_header()` 는 헤더를 `${CMAKE_CURRENT_BINARY_DIR}` 에 만들고, 그 경로를 include 경로에 자동으로 추가합니다. 지금 코드는 빌드 디렉터리가 소스 폴더 아래의 `build` 일 때만 동작합니다. `cmake -B out` 처럼 다른 위치에서 빌드하면 실패합니다.

**제안:** `#include "ultranet.pio.h"` 로 바꿉니다.

### 3. `set_sys_clock_khz()` 반환값 미확인

```c
set_sys_clock_khz(CLOCKSPEED,false);
```

`required=false` 이면 요청한 주파수를 만들 수 없을 때 클럭을 바꾸지 않고 `false` 를 반환합니다. 그러면 125MHz 로 계속 동작해 PIO 샘플링 타이밍(`cy`)과 I2S/MCLK 주파수가 모두 틀어지는데, 아무 표시가 없습니다. 196.5MHz 와 172MHz 는 현재 PLL 로 정확히 만들 수 있지만, `CLOCKSPEED` 를 바꾸면 이 문제가 드러날 수 있습니다.

**제안:** `set_sys_clock_khz(CLOCKSPEED, true)` 로 설정 실패 시 멈추게 하거나, 반환값이 `false` 이면 LED 를 특정 색으로 켜고 멈춥니다. 또한 196.5MHz 는 RP2040 의 원래 정격(133MHz)을 넘으므로, 보드에 따라 `vreg_set_voltage()` 로 코어 전압을 올리는 것도 검토할 만합니다.

## 중간

### 4. `WS2812` 를 끄면 컴파일 실패

`led_state`, `LED_STREAM_MASK`, `LED_STREAM_COLOUR`, `LED_ERR_COLOUR` 는 `#ifdef WS2812` 안에서만 정의됩니다. 하지만 `alarm_callback()` 과 `main()` 의 디코딩 루프는 조건 없이 이들을 사용합니다. 그래서 일반 Pico 에서 `WS2812` 를 끄고 `PICO_LED` 만 켜는 조합은 빌드되지 않습니다.

**제안:** `led_state` 선언과 색상/마스크 정의를 `#ifdef WS2812` 밖으로 옮기고, 실제 LED 출력(`put_pixel`)만 조건부로 둡니다.

### 5. `DEBUG` 만 켜고 `WS2812` 를 끄면 중괄호 불균형

```c
#ifdef DEBUG
    ...
#ifdef WS2812
    while(true)
    {
        ...
#endif // WS2812
    }          // <- WS2812 가 없으면 짝이 없는 닫는 중괄호
#endif // DEBUG
```

`while(true) {` 는 `#ifdef WS2812` 안에 있지만 닫는 `}` 는 밖에 있습니다.

**제안:** 닫는 `}` 를 `#endif // WS2812` 위로 옮깁니다.

### 6. `led_state` 경쟁 조건

코어0 메인 루프의 `led_state = led_state | LED_STREAM_COLOUR;` 는 읽기, OR, 쓰기의 세 단계입니다. 그 사이에 타이머 인터럽트(`alarm_callback`, 같은 코어0)가 들어와 파란 비트를 지우면, 메인 루프가 이전 값을 다시 써서 지운 결과가 사라집니다. 영향은 LED 가 한 주기(200ms) 더 켜져 있는 정도로 작습니다.

**제안:** 표시용 플래그를 별도의 `volatile bool` 로 나누거나, 수정 구간에서 인터럽트를 잠시 막습니다(`save_and_disable_interrupts()`).

### 7. 오류 LED 가 영구히 켜짐 (주석과 동작 불일치)

`LED_STREAM_MASK`(`0xFFFF00FF`)는 파란 비트만 지우므로 빨간 오류 비트는 한 번 켜지면 재부팅 전까지 유지됩니다. 파일 머리 주석은 "오류 빈도를 볼 수 있도록 LED 를 토글한다"고 설명합니다. 의도적인 래치라면 주석을 고치고, 오류 빈도를 보는 것이 목적이라면 빨간 비트도 주기적으로 지우도록 마스크를 바꿉니다.

### 8. 공식 Pico 보드에서 쓸 수 없는 핀

`MCLK_PIN 24` 와 `PIN_PWM_4B 29` 는 공식 Raspberry Pi Pico 에서 내부 용도(VBUS 감지, VSYS 측정)로 연결되어 있어 핀 헤더로 나오지 않습니다. 코드가 GP16 에 WS2812 가 있는 호환 보드를 기준으로 한다는 점을 README 에 명시하거나, 보드별 핀 설정을 분리하는 것이 좋습니다. (`README.ko.md` 에는 반영했습니다.)

### 9. `alarm_callback()` 의 반환값 형변환

```c
const uint64_t repeat_us = STREAM_LED_RESET;
...
return *(const uint32_t*)repeatptr;
```

`uint64_t` 변수를 `uint32_t*` 로 읽습니다. 리틀 엔디언인 RP2040 에서는 우연히 맞는 값이 나오지만 타입이 맞지 않습니다. 또 `main()` 의 지역 변수 주소를 넘기므로 `main()` 이 끝나지 않는다는 가정에 기대고 있습니다.

**제안:** 콜백에서 `return STREAM_LED_RESET;` 로 바로 반환하거나, `static const int64_t` 로 선언하고 같은 타입으로 읽습니다.

### 10. `CMakeLists.txt`: SDK 파일을 `touch`

```cmake
add_custom_target(REBUILD_BINARY_INFO
  COMMAND touch ${PICO_SDK_PATH}/src/rp2_common/pico_standard_link/binary_info.c
)
```

빌드할 때마다 SDK 안의 파일을 수정합니다. SDK 가 읽기 전용이면 실패하고, `touch` 가 없는 Windows 셸에서도 실패합니다. 또 같은 SDK 를 쓰는 다른 프로젝트도 매번 다시 빌드됩니다.

**제안:** `${CMAKE_COMMAND} -E touch` 를 쓰거나, 버전 문자열을 `pico_set_program_version()` 으로 넘기는 방식으로 이 타깃을 없앱니다.

## 낮음

### 11. `ultranet_pio_init()` 가 인자 대신 상수 사용

```c
pio_sm_set_jmp_pin(pio, UNET_SM, pin);
```

다른 호출은 모두 인자 `sm` 을 쓰는데 여기만 `UNET_SM` 을 씁니다. 지금은 값이 같아 문제없지만, 다른 상태 머신으로 호출하면 엉뚱한 SM 의 설정이 바뀝니다. `sm` 으로 바꿉니다.

### 12. `core1_entry()` 정리

- `uint selector = get_selector();` 는 쓰이지 않습니다. 바로 아래에서 `sel_sw` 로 다시 읽습니다.
- `ssample` 은 `volatile` 일 필요가 없고, 주석("signed version")과 달리 부호 없는 오프셋 바이너리 값입니다. `volatile` 을 빼면 루프가 조금 가벼워집니다.
- `ch[count] = (count + (sel_sw<<1)) & 7;` 은 bit2(스트림 선택)가 `<<1` 뒤 8 이 되어 `& 7` 로 지워지는 것에 기대고 있습니다. `((sel_sw & 0b11) << 1)` 로 의도를 드러내는 편이 읽기 쉽습니다.

### 13. 스테레오 쌍의 프레임 불일치 가능성

코어1 은 코어0 이 `samples[]` 를 쓰는 도중에도 읽을 수 있습니다. 32비트 쓰기는 원자적이라 값이 깨지지는 않지만, 한 I2S 프레임의 왼쪽과 오른쪽이 서로 다른 Ultranet 프레임에서 올 수 있습니다(1샘플, 약 21us 차이). 들리지는 않는 수준이라 지금 설계로 충분합니다. 정확한 정렬이 필요하면 더블 버퍼와 프레임 카운터(seqlock)를 쓰면 됩니다.

### 14. `push noblock` 오버런 미감지

Ultranet PIO 는 `push noblock` 을 쓰므로 코어0 이 늦으면 워드가 조용히 버려집니다(`FDEBUG.RXSTALL` 만 설정). 코어0 의 일이 적어 실제로는 거의 일어나지 않지만, 디버그 빌드에서 `pio->fdebug` 의 RXSTALL 비트를 확인하면 원인 분석에 도움이 됩니다.

### 15. 오래되었거나 틀린 주석

| 위치 | 내용 |
|---|---|
| `ultranet.pio` 머리, MCLK, I2S 설명 | 172MHz / 7 기준 설명. 현재 기본값은 196.5MHz / 8 |
| `ultranet.c` `ws2812_pio_init()` | `sm_config_set_out_shift(&c, false, true, 24)` 주석이 "RIGHT, no autopull" 이지만 실제는 왼쪽 시프트, autopull 켜짐 |
| `ultranet.h` 색상 | `CYAN (RED\|BLUE)` 는 실제로 마젠타, `MAGENTA (GREEN\|BLUE)` 는 실제로 시안 (GRB 순서) |
| `ultranet.c` `ultranet_gpio_init()` | `gpio_pull_up` 의 주석이 "switch common to +3.3v" (실제는 0V) |
| `core1.c` `i2s_pio_init()` | "set frequency of UNET_SM" (실제는 I2S SM) |
| `core1.c` | "load i2c output code" (i2s) |
| `README.md` | SDK V2.0.0, 172MHz / 7 설명 (CMake 는 SDK 2.1.0, 코드는 196.5MHz / 8) |

소스 파일의 한국어 주석에는 실제 동작을 기준으로 적었습니다.

### 16. 반복 코드

`main()` 의 서브프레임 읽기 8회와 `core1_entry()` 의 출력 8회는 풀어 쓴 반복문입니다. 타이밍을 예측하기 쉽다는 장점이 있지만, 오프셋 테이블과 짧은 루프로 바꿔도 성능 차이는 거의 없고 수정 실수가 줄어듭니다.

## 잘된 점

- PIO 프로그램에 심볼 위치/래치 시점을 표로 적어 둔 주석이 타이밍 분석에 매우 유용합니다.
- I2S 상태 머신 4개를 FIFO 를 미리 채운 뒤 `pio_set_sm_mask_enabled()` 로 동시에 시작해, 출력 간 위상을 정확히 맞췄습니다.
- MCLK 와 I2S 가 같은 분주비를 써서 MCLK 와 BCLK/LRCLK 가 정확한 정수비를 이룹니다.
- `bi_decl` 로 핀 정보를 바이너리에 넣어 `picotool info` 로 배선을 확인할 수 있습니다.
- 시작 직후 200 워드를 버리고 동기부터 맞춰, 부팅할 때 오류 LED 가 켜지지 않게 한 처리가 깔끔합니다.
