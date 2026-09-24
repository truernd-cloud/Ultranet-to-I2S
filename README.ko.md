# Ultranet-to-I2S (한국어)

[English README](README.md)

Raspberry Pi Pico(RP2040)로 Behringer Ultranet 신호를 디코딩해 I2S 스트림 여러 개와 PWM 아날로그 출력으로 내보내는 펌웨어입니다.

## 개요

Behringer Ultranet 에서 아날로그 오디오를 간단하고 저렴하게 뽑아내는 것이 이 프로젝트의 목표입니다. Raspberry Pi Pico 에는 이 작업에 딱 맞는 기능이 두 가지 있습니다.

- **PIO 모듈**: 들어오는 Ultranet 스트림을 오디오 8채널로 디코딩하고, 채널 두 개씩 묶어 I2S 출력 스트림으로 인코딩합니다. I2S 출력은 PCM5102A 같은 저가 DAC 모듈에 바로 연결할 수 있습니다.
- **듀얼 코어**: Ultranet 입력은 송신 장비의 클럭을, I2S 출력은 이 Pico 의 클럭을 따릅니다. 두 클럭은 비슷하게 맞출 수는 있어도 완전히 같을 수는 없습니다. 코어0 은 Ultranet 샘플을 배열에 채우고, 코어1 은 그 배열을 읽어 I2S 로 내보냅니다. 두 코어가 서로 독립적으로 동작하므로, 동기가 어긋난 디지털 오디오에서 흔히 생기는 클릭 노이즈가 생기지 않습니다.

클럭 차이는 샘플을 한 번 더 출력하거나 하나 건너뛰는 식으로 흡수됩니다. 두 클럭이 충분히 가까우면 이런 보정은 드물게 일어나고, 인접 샘플의 크기가 거의 같아 귀로는 들리지 않습니다.

## 동작 원리

| 항목 | 값 |
|---|---|
| Ultranet 비트 레이트 | 12.288MHz (32비트 x 8채널 x 48kHz) |
| 바이페이즈 선로 클럭 | 24.576MHz |
| 시스템 클럭 (기본값) | 196.5MHz (선로 클럭의 약 8배로 샘플링) |
| 대안 시스템 클럭 | 172MHz (약 7배로 샘플링) |
| 오디오 샘플 | 22비트 (Ultranet 실제 비트 깊이) |
| I2S 출력 | 스테레오 4개, 48kHz, 32비트 슬롯 |
| MCLK | 약 12.28MHz (256 x fs) |
| PWM 출력 | 모노 8개, 12비트, 약 48kHz |

클럭을 바꾸려면 다음 세 곳을 함께 수정해야 합니다.

- `ultranet.h`: `CLOCKSPEED` (196500 또는 172000), `AUDIV` (8 또는 7)
- `ultranet.pio`: `.define cy` (8 또는 7)

### 소스 파일 구성

| 파일 | 내용 |
|---|---|
| `ultranet.h` | 공용 헤더: 핀 배치, 클럭, 옵션 스위치, LED 색상 정의 |
| `ultranet.c` | 코어0: 초기화, Ultranet 프레임 동기 및 샘플 디코딩, 상태 LED |
| `core1.c` | 코어1: I2S/MCLK/PWM 초기화, 샘플을 I2S 와 PWM 으로 출력 |
| `ultranet.pio` | PIO 프로그램: Ultranet 수신기, MCLK, I2S 송신기, WS2812 LED 드라이버 |

## 핀 배치

| GPIO | 기능 |
|---|---|
| GP0 | Ultranet 하위 스트림(채널 1-8) 입력 |
| GP1 | Ultranet 상위 스트림(채널 9-16) 입력 |
| GP2 / GP3 / GP4 | I2S 1: DATA / BCLK / LRCLK |
| GP5 / GP6 / GP7 | I2S 2: DATA / BCLK / LRCLK |
| GP8 / GP9 / GP10 | I2S 3: DATA / BCLK / LRCLK |
| GP17 / GP18 / GP19 | I2S 4: DATA / BCLK / LRCLK |
| GP11 / GP12 / GP13 | 셀렉터 스위치 bit0 / bit1 / bit2 |
| GP14 / GP15 | PWM 1 왼쪽 / 오른쪽 |
| GP20 / GP21 | PWM 2 왼쪽 / 오른쪽 |
| GP26 / GP27 | PWM 3 왼쪽 / 오른쪽 |
| GP28 / GP29 | PWM 4 왼쪽 / 오른쪽 |
| GP16 | WS2812 상태 LED |
| GP24 | I2S MCLK 출력 |

> **주의:** 공식 Raspberry Pi Pico 보드에서는 GP24(VBUS 감지)와 GP29(VSYS 측정)가 내부에 연결되어 있어 핀 헤더로 나오지 않습니다. 이 핀 배치는 GP16 에 WS2812 LED 가 있는 소형 중국산 RP2040 보드를 기준으로 합니다. 공식 Pico 를 쓴다면 `ultranet.h` 에서 `MCLK_PIN` 과 `PIN_PWM_4B` 를 사용 가능한 핀으로 바꾸거나 해당 기능을 끄세요.

PWM 출력을 아날로그 오디오로 쓰려면 각 핀에 RC 저역 통과 필터를 달아야 합니다.

## 셀렉터 스위치

3비트 이진 스위치(GP11~13)로 입력 스트림과 출력 채널 배치를 고릅니다. 스위치는 **부팅할 때 한 번만** 읽으므로, 위치를 바꾼 뒤에는 리셋해야 합니다.

- 기본 설정(`SW_COMM_LOW`)은 스위치 공통 단자를 0V 에 연결하고 내부 풀업을 사용합니다. 스위치가 ON 이면 해당 비트가 1 입니다.
- 공통 단자를 3.3V 에 연결하려면 `ultranet.h` 에서 `SW_COMM_LOW` 대신 `SW_COMM_HIGH` 를 정의하세요.
- 스위치를 달지 않으면 모든 비트가 0 이 되어 스트림 1-8, 기본 채널 배치로 동작합니다.

**bit2**: 입력 스트림 선택 (0 = 채널 1-8, GP0 / 1 = 채널 9-16, GP1)

**bit1..0**: 출력 채널 쌍 오프셋 (bit2 가 1 이면 아래 채널 번호에 8 을 더합니다)

| bit1..0 | I2S 1 / PWM 1 | I2S 2 / PWM 2 | I2S 3 / PWM 3 | I2S 4 / PWM 4 |
|---|---|---|---|---|
| 00 | 1-2 | 3-4 | 5-6 | 7-8 |
| 01 | 3-4 | 5-6 | 7-8 | 1-2 |
| 10 | 5-6 | 7-8 | 1-2 | 3-4 |
| 11 | 7-8 | 1-2 | 3-4 | 5-6 |

그래서 I2S 1 출력 하나만 달아도 스위치 조합으로 16채널 중 원하는 스테레오 쌍을 고를 수 있습니다.

## 상태 LED (WS2812)

| 색 | 의미 |
|---|---|
| 파랑 | Ultranet 스트림 정상 수신 중. 스트림이 끊기면 약 200ms 안에 꺼집니다. |
| 빨강 | 프레임 동기 오류가 한 번 이상 발생. 재부팅 전까지 계속 켜져 있습니다. |
| 보라(파랑+빨강) | 수신 중이며 과거에 동기 오류가 있었음 |

일반 Pico 보드의 기본 LED(GP25)를 쓰려면 `ultranet.h` 의 `#define PICO_LED 25` 주석을 해제하세요. 현재 코드는 `WS2812` 정의가 있어야 컴파일되므로 `WS2812` 는 켜 둔 채로 사용해야 합니다([코드 리뷰](CODE_REVIEW.ko.md) 참고).

## 하드웨어

Ultranet 케이블의 차동 신호를 Pico 가 받을 수 있는 단일 종단 TTL 신호로 바꾸려면 라인 리시버 IC 가 필요합니다. 제작자의 시제품은 Ultranet 분배기용으로 만든 PCB 에서 입력 버퍼 IC(MC3486)와 5V 레귤레이터만 쓰고, 남은 공간에 Pico 호환 보드와 PCM5102A DAC 모듈을 달았습니다.

![PicoUltranetDecoder](https://github.com/user-attachments/assets/e9e0c4d6-15a5-412d-a558-9ded5bb5acc3)

입력 회로도: [Schematic_UltranetInput_2025-01-01-2.pdf](https://github.com/user-attachments/files/18286932/Schematic_UltranetInput_2025-01-01-2.pdf)

## 빌드

개발 환경은 Raspberry Pi Pico C/C++ SDK 와 Visual Studio Code(Raspberry Pi Pico 확장)입니다. `CMakeLists.txt` 는 SDK 2.1.0, 툴체인 13_3_Rel1 을 지정합니다.

VS Code 확장을 쓰면 프로젝트를 열고 **Compile Project** 를 실행하면 됩니다. 명령줄에서 빌드하려면 다음과 같이 합니다.

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
mkdir build && cd build        # 빌드 디렉터리 이름은 반드시 "build" 여야 합니다
cmake ..
make
```

빌드가 끝나면 `build/Ultranet.uf2` 가 생깁니다. Pico 의 BOOTSEL 버튼을 누른 채 USB 에 연결한 뒤 나타나는 드라이브에 이 파일을 복사하면 됩니다.

> **알려진 빌드 문제**
> - `CMakeLists.txt` 는 소스 파일을 `Ultranet.c` 로 지정하지만 실제 파일 이름은 `ultranet.c` 입니다. Windows 에서는 문제없지만, Linux 처럼 대소문자를 구분하는 파일 시스템에서는 파일을 찾지 못해 빌드가 실패합니다. `CMakeLists.txt` 의 파일 이름을 `ultranet.c` 로 고쳐서 빌드하세요.
> - `ultranet.h` 가 `build/ultranet.pio.h` 를 직접 include 하므로 빌드 디렉터리가 소스 폴더 바로 아래의 `build` 여야 합니다.
> - `REBUILD_BINARY_INFO` 타깃이 SDK 안의 파일을 `touch` 하므로 SDK 폴더에 쓰기 권한이 있어야 합니다.

## 오버클럭 관련

기본 설정은 RP2040 을 196.5MHz 로 오버클럭합니다. 제작자는 여러 중국산 호환 보드에서 문제없이 동작하는 것을 확인했습니다. 동작이 불안정하면 172MHz 설정(위 "동작 원리" 참고)을 시도해 보세요.

## 참고한 프로젝트

- [elehobica/pico_spdif_rx](https://github.com/elehobica/pico_spdif_rx): Ultranet 이 AES/EBU 와 매우 비슷하므로 이 프로젝트의 PIO 수신 프로그램을 수정해 사용했습니다.
- [nyh-workshop/rpi-pico-i2sExample](https://github.com/nyh-workshop/rpi-pico-i2sExample): I2S 출력 PIO 프로그램을 가져와 사용했습니다.

## 라이선스

[LICENSE](LICENSE) 파일을 참고하세요.
