# 블라인드형 실시간 복사열 감지 기반 가변형 자동 차광 시스템

이 폴더는 Arduino Nano ESP32용 최종 업로드 패키지입니다.

> **V3 상단 원점센서 전용 최종판:** Upper Endstop만 D4에 연결합니다. **D5와 Lower Endstop은 사용하지 않습니다.** 하단은 실제 측정값의 92%를 `DEPLOY_STEPS`로 저장한 소프트웨어 한계로 제어합니다. DS18B20 DATA는 D2이며 `Pin Numbering`은 반드시 **`By GPIO number (legacy)`**로 선택하십시오.

- `SMART_SHADE_VARIABLE_FINAL.ino`: Arduino IDE에서 여는 완성 스케치
- `FINAL_CHECKLIST_KO.txt`: 업로드 직후부터 AUTO 직전까지의 짧은 최종 점검표
- `WIRING_TABLE.csv`: 결선 작업용 표
- `LIBRARIES.txt`: 필요한 라이브러리와 확인된 버전

> **최우선 안전 경고**  
> Nano ESP32의 모든 GPIO와 ADC 입력은 3.3 V 전용입니다. 센서 신호, Endstop의 S, I²C의 SDA/SCL에 5 V가 들어가면 보드가 손상될 수 있습니다. 모터용 외부 5 V의 `+`는 ULN2003에만 연결하고 Nano의 3V3 핀에는 절대 연결하지 마십시오. 단, 외부 전원 `GND`와 Nano `GND`는 반드시 공통으로 연결합니다.

## STEP 1. 하드웨어 구성 검토

구성은 논리적으로 적절합니다. 센서 입력과 구동 출력이 서로 겹치지 않으며, A4/A5는 I²C, A0/A1은 ADC로 사용할 수 있습니다. D2는 1-Wire, D3은 온습도, D4는 Upper Endstop, D6~D9는 ULN2003용입니다. D5는 비워 둡니다.

Arduino IDE에서 다음 설정을 사용합니다.

- 보드: `Arduino Nano ESP32`
- Pin Numbering: **`By GPIO number (legacy)` 필수**
- 코드에는 `D2`~`D9` 별칭을 사용하므로 실제 결선은 Nano 보드에 인쇄된 D2~D9 그대로입니다. 이 설정은 OneWire 2.3.8이 Nano ESP32의 기본 Arduino 핀 리매핑을 올바르게 처리하지 못하는 문제를 피하기 위해 필요합니다.

차광막 위치 정의는 다음과 같습니다.

- 0%: 완전히 올라감, Upper Endstop 위치
- 100%: 물리적 최대 전개길이보다 8% 짧게 저장된 소프트웨어 하단 위치
- 임의 위치: `DEPLOY_STEPS × percent / 100`

28BYJ-48은 엔코더가 없는 개방루프 모터이므로 실제로 걸리거나 미끄러졌을 때 코드가 그 손실 스텝을 직접 알 수는 없습니다. 그래서 전원을 켤 때마다 Upper Endstop으로 Homing합니다. 조립 후 `CALIBRATE`로 원점에 복귀하고 `JOG` 명령으로 실제 최대 전개길이를 측정한 뒤 `CAL SET`으로 그 값의 92%를 저장합니다. 레일 하단에는 반드시 봉 이탈 방지용 물리적 막음캡을 설치합니다.

확정할 수 없는 부분은 모듈 PCB마다 회로가 다르다는 점입니다. 특히 GUVA, 조도센서, RAMPS Endstop, MLX90614 모듈의 정확한 판매처·회로도가 없으므로, 아래 전압 검사를 통과한 모듈만 연결해야 합니다.

## STEP 2. 전기적 안전성 검토

### 전원 구조

```text
PC/USB ───────────────> Nano ESP32
                            │
                            └── GND ─────────────┐
                                                 │ 공통 GND
외부 안정화 5 V 전원 ── +5V ──> ULN2003 +       │
                      └─ GND ──> ULN2003 - ──────┘

Nano D6~D9 ────────────────> ULN2003 IN1~IN4
ULN2003 모터 커넥터 ────────> 28BYJ-48
```

- Nano와 모터는 양(+) 전원을 따로 받습니다.
- 제어 신호의 기준 전압을 같게 만들기 위해 GND만 공통으로 묶습니다.
- 외부 5 V `+`를 Nano의 `3V3`, GPIO, 센서 출력에 연결하지 마십시오.
- 모터 전원은 안정화된 5 V, 권장 여유 용량 1 A 이상을 사용하십시오. 실제 필요한 전류는 보유한 모터 명판과 전원 사양을 우선합니다.
- 모터는 Nano 핀에 직접 연결하지 않고 반드시 ULN2003을 거칩니다.

### 부품별 판정

| 부품 | 판정과 연결 전 확인사항 |
|---|---|
| Nano ESP32 | GPIO/ADC는 3.3 V 전용입니다. USB로 보드에 전원을 공급합니다. |
| ULN2003A | 정품 ULN2003A 계열이라면 3.3 V GPIO 제어가 일반적인 28BYJ-48 부하에서 가능합니다. 모듈의 실제 IC 표기를 확인하십시오. |
| GUVA 모듈 | 우선 3V3로 구동하고 `OUT-GND`를 멀티미터로 측정하십시오. 모든 조명 조건에서 0~3.3 V여야 합니다. 3.3 V에서 동작하지 않는 모듈이면 바로 5 V로 바꾸지 말고 레벨 변환/분압을 설계해야 합니다. |
| 3핀 조도센서 | 단자가 `OUT/VCC/GND`라면 `OUT`이 이 설계의 `AO`입니다. 3V3 구동 후 OUT이 0~3.3 V인지 확인합니다. 출력이 밝을수록 감소하는 모듈도 있으므로 실제 Raw 변화로 방향을 확인합니다. |
| DS18B20 | 3V3 구동 가능합니다. DATA와 3V3 사이에 4.7 kΩ pull-up이 필요합니다. 모듈에 이미 실장되어 있으면 외부 저항을 중복 연결하지 않아도 됩니다. |
| DHT11 | 3V3 구동을 우선합니다. 모듈형은 DATA pull-up이 VCC로 연결되므로 5 V로 구동하면 Nano DATA 핀에 5 V가 들어갈 수 있습니다. |
| RAMPS Endstop | 정확한 PCB 회로가 불명확합니다. VCC를 써야 하는 모듈은 3V3만 사용하고, S의 HIGH가 3.3 V 이하인지 측정하십시오. 가장 단순하고 안전한 방식은 기계식 스위치의 COM/GND와 NO/S 두 선만 사용하고 Nano 내부 pull-up을 쓰는 것입니다. |
| MLX90614 | 센서/브레이크아웃에 3 V형, 5 V 대응형, 서로 다른 pull-up 구성이 있습니다. 모듈 사양을 확인해 3V3 호환이면 3V3에 연결합니다. 5 V 전원이 필수라면 SDA/SCL에 양방향 I²C 레벨 시프터를 넣고 Nano 쪽 pull-up은 3.3 V로 둡니다. |

연결 전 멀티미터 검사 순서:

1. Nano와 센서 전원을 끈 상태에서 5 V와 3V3 사이 단락이 없는지 확인합니다.
2. Nano에서 분리한 채 각 모듈에 계획된 전압을 공급합니다.
3. GUVA OUT, Light OUT, Endstop S, MLX SDA/SCL의 최대 전압을 측정합니다.
4. 모두 3.3 V 이하일 때만 Nano에 신호선을 연결합니다.

## STEP 3. 최종 결선표

| 장치 | 장치 단자 | Nano/전원 연결 | 비고 |
|---|---|---|---|
| GUVA-S12SD3528 | OUT 또는 AO | A0 | 사용자가 가진 3핀 모듈의 `OUT`이 아날로그 출력 |
|  | VCC | 3V3 | 먼저 모듈의 3.3 V 동작 여부 확인 |
|  | GND | GND | 공통 GND |
| 조도센서 3핀 | OUT | A1 | 이 설계에서 AO와 같은 의미, DO는 사용하지 않음 |
|  | VCC | 3V3 | OUT 최대 3.3 V 확인 |
|  | GND | GND | 공통 GND |
| DS18B20 | DATA | D2 | DATA-3V3 사이 4.7 kΩ pull-up, 보드에 있으면 생략 가능 |
|  | VCC | 3V3 |  |
|  | GND | GND |  |
| DHT11 | DATA/S | D3 |  |
|  | VCC | 3V3 |  |
|  | GND | GND |  |
| Upper Endstop | S | D4 | 기본 `눌림=LOW` |
|  | -/GND | GND |  |
|  | +/VCC | 3V3 또는 미사용 | 모듈 회로 확인 후 결정, 5 V 금지 |
| Lower Endstop | 사용하지 않음 | D5 연결 금지/비워 둠 | 하단은 `DEPLOY_STEPS`와 물리적 막음캡으로 제한 |
| ULN2003 | IN1 | D6 |  |
|  | IN2 | D7 |  |
|  | IN3 | D8 |  |
|  | IN4 | D9 |  |
|  | +/VCC | 외부 5 V + | Nano 3V3에서 모터를 공급하지 않음 |
|  | -/GND | 외부 5 V GND 및 Nano GND | 반드시 공통 GND |
| 28BYJ-48 | 5핀 플러그 | ULN2003 모터 소켓 | 키 방향 확인 |
| MLX90614 | SDA | A4/SDA | I²C pull-up이 3.3 V보다 높지 않아야 함 |
|  | SCL | A5/SCL | I²C pull-up이 3.3 V보다 높지 않아야 함 |
|  | VCC/VIN | 모듈 사양에 따름 | 3V3 호환 여부를 확인; 5 V형이면 레벨 시프터 필요 |
|  | GND | GND | 공통 GND |
| Nano ESP32 | USB | PC 또는 USB 전원 | 모터 5 V와 양(+) 전원 분리 |

AccelStepper의 28BYJ-48 HALF4WIRE 생성자에는 코일 순서 때문에 코드상 `IN1, IN3, IN2, IN4` 순서로 전달합니다. 실제 결선 자체는 표대로 `D6→IN1, D7→IN2, D8→IN3, D9→IN4`입니다.

## STEP 4. 전체 제어 알고리즘

```text
GUVA + 조도 + DS18B20 + DHT11 + MLX90614
                     ↓ 필터링/유효성 검사
각 센서값을 0~1로 정규화
                     ↓
EEI = 유효한 센서의 가중 평균
                     ↓
야간 판정 + 단계 임계값 + 히스테리시스
                     ↓
목표 0/25/50/75/100% (옵션으로 연속 0~100%)
                     ↓ 10초 안정시간·최소 10% 변화 확인
DEPLOY_STEPS 기준 목표 스텝 계산
                     ↓
AccelStepper 비차단 이동 + 매 loop Upper/STOP/timeout/소프트웨어 한계 확인
```

초기 EEI 식은 다음과 같습니다.

```text
0.18×UV + 0.18×Light + 0.10×OutdoorTemp
+ 0.20×IndoorTemp + 0.27×WindowTemp + 0.07×Humidity
```

이 수치들은 모두 **실험 전 초기 테스트값**이며 검증된 과학 상수가 아닙니다. MLX 창문 항목은 창문 절대온도 70%와 `창문온도-외부그늘온도` 차이 30%를 조합합니다. 일부 비핵심 센서가 일시적으로 빠지면 남은 유효 weight로 다시 나누지만, AUTO에는 실내온도, MLX 창문온도, GUVA/조도 중 하나 이상이 반드시 필요합니다.

- 밤: UV와 조도가 모두 낮은 상태가 30초 지속되면 0% 개방 목표
- 단계 제어: EEI 기준 0/25/50/75/100%
- 히스테리시스: 각 경계 ±0.03
- 안정시간: 새 목표가 10초 유지된 뒤 이동
- 최소 변화: 현재 위치와 10% 미만 차이면 이동하지 않음
- 연속 제어 확장: `EEI_CONTINUOUS_POSITION = true`이면 기본적으로 `EEI×100%`

## STEP 5. State Machine

| 상태 | 의미 | 정상 전환 |
|---|---|---|
| STARTUP | 시작 직후 | 초기화 후 HOMING |
| HOMING | Upper Endstop 방향 저속 이동 | 성공→IDLE 또는 측정 준비, 실패→ERROR |
| IDLE | MANUAL 정지 또는 JOG 측정 준비 상태 | 명령에 따라 HOMING/MOVING/AUTO_IDLE |
| MOVING_UP | 목표 위치로 상승 | 목표/Upper 도달→IDLE 또는 AUTO_IDLE |
| MOVING_DOWN | 목표 위치 또는 측정 JOG만큼 하강 | 목표 스텝 도달→IDLE 또는 AUTO_IDLE |
| AUTO_IDLE | EEI를 감시하며 정지 | 목표 승인→MOVING, `MANUAL`→IDLE |
| ERROR | Homing/이동 시간초과 또는 계산 위치 오류 | 원인 제거 후 `HOME`/`RESET` |
| EMERGENCY_STOP | `STOP`으로 즉시 정지 | 위치 기준을 잃은 것으로 처리, `HOME` 필요 |

모터 이동은 긴 `delay()` 없이 `stepper.run()`을 반복 호출합니다. 각 loop에서 Serial 명령과 Endstop을 모터 스텝보다 먼저 검사합니다.

## STEP 6. 필요한 Arduino 라이브러리

Arduino IDE의 `도구 > 라이브러리 관리...`에서 아래 검색 이름을 그대로 사용합니다.

| 검색/설치 이름 | 제공자 | 용도 |
|---|---|---|
| OneWire | Paul Stoffregen | DS18B20 1-Wire 통신 |
| DallasTemperature | Miles Burton | DS18B20 온도 읽기 |
| DHT sensor library | Adafruit | DHT11 온습도 읽기 |
| Adafruit Unified Sensor | Adafruit | DHT 라이브러리 의존성 |
| Adafruit MLX90614 Library | Adafruit | MLX90614 Object/Ambient 온도 |
| Adafruit BusIO | Adafruit | Adafruit 센서 라이브러리 의존성 |
| AccelStepper | Mike McCauley | 비차단 스텝모터 가감속/위치 제어 |

`Wire`와 `Preferences`는 Nano ESP32 보드 코어에 포함되어 별도 설치하지 않습니다. 이 패키지는 Arduino ESP32 Boards `2.0.18-arduino.5`, AccelStepper `1.64.0` 및 `LIBRARIES.txt`에 적힌 설치 버전으로 실제 컴파일했습니다.

## STEP 7. 전체 Arduino 코드

완성 코드는 같은 폴더의 `SMART_SHADE_VARIABLE_FINAL.ino` 한 파일입니다. 함수 생략이나 의사코드가 아니며 Arduino Nano ESP32 대상으로 컴파일을 완료했습니다.

컴파일 결과:

```text
Sketch uses 354777 bytes (11%) of program storage space.
Global variables use 31844 bytes (9%) of dynamic memory.
```

Arduino IDE에서 반드시 **스케치 폴더와 ino 파일의 이름을 동일하게 유지**한 채 파일을 열고 업로드하십시오.

## STEP 8. 코드에서 사용자가 확인·수정해야 하는 값

`.ino` 맨 위 `USER CONFIGURATION`만 우선 확인합니다.

| 설정 | 초기값 | 언제 변경하는가 |
|---|---:|---|
| `MOTOR_DIRECTION_INVERT` | false | UP이 실제로 내려가면 true로 변경 후 재업로드 |
| `ENDSTOP_ACTIVE_LOW` | true | `ENDSTOPS`에서 눌림/해제 값이 반대이면 변경 |
| `AUTO_START_ENABLED` | false | 전체 검증이 끝난 뒤에만 true 고려 |
| `HOLD_MOTOR_WHEN_IDLE` | false | 정지 시 위치가 하중에 밀릴 때만 true 고려; 발열 확인 필수 |
| `EEI_CONTINUOUS_POSITION` | false | 단계 제어 검증 후 연속 제어로 바꿀 때 true |
| 속도/가속도 | 180~400 step/s | 탈조·진동·토크 부족 시 낮춤 |
| `DEPLOY_SAFETY_FACTOR` | 0.92 | 측정한 물리적 최대 전개길이보다 8% 짧게 저장 |
| 측정/Homing 한계 | 12000/14000 step | 실제 구조가 이 범위를 초과할 때만 안전 검증 후 조정 |
| 정규화 범위 | TEST VALUE | `LOG ON` 실측 데이터로 반드시 보정 |
| EEI weight/threshold | TEST VALUE | 실험 결과로 반드시 보정 |
| 히스테리시스/안정시간/최소 변화 | 0.03/10초/10% | 잦은 왕복이나 느린 반응을 조절할 때 변경 |

`HOMING_MAX_STEPS`, `CALIBRATION_MEASURE_MAX_STEPS`, timeout을 무작정 크게 만들면 고장 시 기구를 오래 밀 수 있습니다. V3 기본값은 이전 약 5,766스텝 시험값을 참고해 측정 12,000스텝, Homing 14,000스텝으로 제한했습니다.

## STEP 9. 최초 구동 절차

Serial Monitor 설정은 `115200 baud`, 줄 끝은 `Newline` 또는 `Both NL & CR`입니다. 각 단계가 성공하기 전에는 다음 단계로 넘어가지 마십시오.

### TEST 1 — 모터 전원 연결 전, 센서만

- 준비: 외부 모터 5 V의 `+`를 분리합니다. Nano와 센서만 연결합니다.
- 부팅 Homing 처리: Upper Endstop을 누른 상태에서 리셋해 즉시 Homing을 끝내거나, 부팅 후 바로 `STOP`을 입력합니다.
- 입력: `SENSORS`, `STATUS`, 필요하면 `LOG ON`.
- 정상: Raw가 환경에 따라 변하고, DS/DHT/MLX가 합리적인 온도를 냅니다.
- 즉시 중단: Nano 핀이 3.3 V를 넘거나 모듈/선이 뜨거워짐.
- 해결: 전원 제거 후 VCC/GND/신호 전압과 공통 GND를 재확인합니다.

### TEST 2 — Endstop

- 모터 5 V는 계속 분리합니다.
- 입력: 누르지 않은 상태와 손으로 누른 상태에서 각각 `ENDSTOPS`.
- 정상: Upper가 `0→1`로 바뀝니다. 출력에는 Lower 항목이 없습니다.
- 중단: 스위치를 누를 때 5 V가 D4에 나타남.
- 해결: S/GND 순서, 3V3 전원, `ENDSTOP_ACTIVE_LOW`를 확인합니다.

### TEST 3 — 모터 및 Upper 방향

- 차광막 이동 경로를 비우고 Nano가 MANUAL인지 확인한 뒤 외부 5 V를 연결합니다.
- 입력: `HOME`.
- 정상: 천천히 감기는 Upper 방향으로 움직이고 Upper를 누르면 즉시 멈춥니다.
- 즉시 중단: 반대 방향, 떨림, 큰 소음, 과열 또는 줄 엉킴. `STOP` 후에는 다시 HOME이 필요합니다.
- 해결: `MOTOR_DIRECTION_INVERT`, D6~D9와 IN1~IN4, 공통 GND를 확인합니다.

### TEST 4 — UP/DOWN 방향

- 보정 전에는 무제한 `DOWN`을 사용하지 않습니다. 아래 TEST 7의 `JOG 100`으로 내려가는 방향을 확인합니다.
- 정상: HOME/UP은 감기고, 양수 JOG는 풀리는 방향입니다.
- 즉시 중단: 반대 방향이면 `STOP`.
- 해결: `MOTOR_DIRECTION_INVERT` 값을 반대로 바꾸고 재업로드합니다. ULN 결선을 임의로 뒤섞어 방향을 바꾸지 마십시오.

### TEST 5 — Upper Endstop + Motor

- 입력: `HOME` 이동 중 Upper를 눌러 봅니다.
- 정상: debounce 약 20 ms 후 즉시 정지하고 `HOMING COMPLETE`가 나타납니다.
- 즉시 중단: 누른 채 계속 움직임.
- 해결: 전원 제거 후 `ENDSTOPS` 값, S 핀, Active LOW 설정을 다시 검사합니다.

### TEST 6 — HOME

- 기구를 실제 차광막에 연결하고 이동 경로를 비웁니다.
- 입력: `HOME`.
- 정상: 천천히 상승, Upper에서 정지, `Current Steps=0`, `Position=0%`, `State=IDLE`.
- 즉시 중단: 걸림, Upper를 지나 밀기, 90초 이내 미도달.
- 해결: 방향, Upper 위치, 속도, `HOMING_MAX_STEPS`와 timeout을 확인합니다.

### TEST 7 — CALIBRATE

- 먼저 Upper HOME과 레일 하단 물리적 막음캡을 확인합니다.
- `CALIBRATE`를 입력하고 `MEASUREMENT READY`가 나올 때까지 기다립니다.
- `JOG 1000`으로 조금씩 내리고 하단 부근에서는 `JOG 100`, `JOG 50`을 사용합니다. 잘못 내려갔으면 `JOG -100`으로 올립니다.
- 원하는 물리적 최대 전개 위치에 도달하면 `CAL SET`을 입력합니다.
- 정상: 측정값과 그 92%인 `DEPLOY_STEPS`를 저장하고 자동으로 Upper에 복귀합니다.
- 즉시 중단: 하단 막음캡을 밀기 시작함, 케이블 감김, 탈조 또는 과열. `STOP` 후에는 HOME부터 다시 시작합니다.

### TEST 8 — 위치 제어

- 입력 순서: `POS 25`, `POS 50`, `POS 75`, `POS 100`, `POS 0`, 마지막으로 `POS 37`.
- 정상: `Current Position`과 실제 전개율이 대략 일치하고 목표마다 정지합니다.
- 즉시 중단: 물리적 하단 막음캡 충돌, 줄/차광막 미끄러짐, 위치 오차 급증.
- 해결: 다시 HOME/CALIBRATE하고 장력·커플러·축 미끄러짐을 수정합니다.

### TEST 9 — 센서 데이터

- 입력: `LOG ON`.
- 조건: 직사광, 약한 햇빛, 그늘을 차례로 만들고 차광막도 각 위치에 둡니다.
- 정상: GUVA/Light/창문온도가 물리적 조건에 맞게 변하고 온도는 급격한 비현실 값이 없습니다.
- 중단: ADC 0/4095 고정, MLX 비현실 온도, 센서 경고 반복.
- 해결: 배선과 센서 방향을 수정하고 AUTO는 사용하지 않습니다.

### TEST 10 — EEI

- 입력: `SENSORS` 또는 `LOG ON`.
- 정상: 강한 직사광·뜨거운 창문·높은 실내온도에서 EEI가 커지고, 그늘/밤에 작아집니다.
- 중단: 반대로 움직이거나 한 센서만으로 0↔1이 반복됨.
- 해결: 특히 조도 출력 방향과 각 MIN/MAX를 보정합니다.

### TEST 11 — AUTO

- TEST 1~10을 모두 통과하고 `STATUS`에서 `AUTO data ready=1`인지 확인합니다.
- 입력: `AUTO`.
- 정상: 새 목표가 10초 유지된 뒤 한 번 이동합니다. 단계 목표는 0/25/50/75/100%입니다. 밤 조건은 30초 후 0%를 목표로 합니다.
- 즉시 중단: 예상 밖 방향, 짧은 주기 반복, 기구 충돌 위험. `STOP`은 즉시 정지하고 위치 기준을 무효화하므로 이후 `HOME`이 필요합니다.
- 해결: 먼저 `MANUAL`, 필요 시 `HOME`; EEI 범위와 안정화 설정을 다시 조정합니다.

## STEP 10. AUTO 모드 테스트

AUTO는 단순히 밝다고 즉시 닫지 않습니다. 다음 시나리오를 각 2~5분 유지하며 `LOG ON` 결과와 목표를 기록하십시오.

| 시나리오 | 예상 경향 |
|---|---|
| 강한 직사광 + 뜨거운 유리 + 더운 실내 | EEI 상승, 75~100% 방향 |
| 강한 빛 + 비교적 차가운 실내/유리 | 중간 위치 가능; 바로 100%가 되지 않는지 관찰 |
| 그늘 + 차가운 유리 | EEI 하강, 0~25% 방향 |
| UV/조도 모두 매우 낮은 밤 | 30초 확인 후 0% 개방 |
| 경계값 근처에서 약간의 빛 변화 | 히스테리시스와 10초 안정시간으로 잦은 왕복이 없어야 함 |

AUTO 중 반드시 사람이 옆에서 처음 여러 회의 전체 왕복을 관찰하십시오. `AUTO_START_ENABLED`는 충분히 검증할 때까지 false로 둡니다.

## STEP 11. EEI 실험 및 보정 방법

1. `LOG ON`으로 CSV를 켜고 직사광 강/약, 그늘, 밤 조건을 각각 기록합니다.
2. 각 조건에서 `POS 0/25/50/75/100`을 사용하고 온도가 안정된 구간을 최소 수분씩 저장합니다.
3. GUVA와 조도는 낮은 복사열 조건의 대표 Raw를 `*_MIN`(정규화 0), 높은 조건의 대표 Raw를 `*_MAX`(정규화 1)로 둡니다. 밝을수록 Raw가 낮아지는 조도 모듈이면 첫 값이 둘째 값보다 커도 코드가 역방향 정규화를 처리합니다.
4. 온도도 같은 방식으로 실제 관측 범위를 넣습니다. 극단 한 번보다 안정 구간의 5~95 백분위 같은 대표 범위를 쓰면 노이즈에 덜 민감합니다.
5. 모든 weight 합을 1.00으로 유지하고 한 번에 하나의 weight만 바꿉니다. 창문 복사열 반응이 약하면 `W_WINDOW`, 태양 입력 반응이 약하면 `W_UV/W_LIGHT`, 실내 쾌적성 반영이 약하면 `W_IN_TEMP`를 조금씩 조정합니다.
6. 로그에서 원하는 차광 단계가 바뀌는 EEI 분포를 보고 `EEI_LEVEL_1..4`를 정합니다.
7. 경계 왕복이 있으면 `EEI_HYSTERESIS`, `CONTROL_STABLE_TIME_MS`, `MIN_POSITION_CHANGE_PERCENT`를 늘립니다. 반응이 지나치게 느리면 한 항목씩 작게 줄입니다.
8. 단계 제어가 안정된 뒤에만 `EEI_CONTINUOUS_POSITION=true`를 시험합니다.

정규화와 weight를 바꾼 뒤에는 같은 시나리오를 다시 실행해 이전 CSV와 비교하십시오. `DEPLOY_STEPS`는 측정한 물리적 전개길이의 92%이며 EEI 보정과 별개입니다.

## STEP 12. Troubleshooting

| 증상 | 확인 및 해결 |
|---|---|
| Nano ESP32가 센서를 인식하지 못함 | 보드/포트/Pin Numbering을 확인하고, 공통 GND와 3V3를 측정합니다. 한 센서씩 분리해 단락이나 버스 방해를 찾습니다. |
| MLX90614 I²C 인식 실패 | A4=SDA, A5=SCL 교차 여부, 기본 주소 0x5A, 모듈 전압, pull-up 전압을 확인합니다. 5 V pull-up이면 Nano에서 즉시 분리하고 레벨 시프터를 사용합니다. |
| MLX90614 창문 온도가 이상함 | Object가 유리를 정확히 보도록 시야각·거리·반사를 조정합니다. 센서 자신이나 프레임을 보지 않는지, 유리 반사 때문에 다른 열원을 보는지 확인합니다. |
| DS18B20이 -127°C | 단선/주소 인식 실패입니다. DATA 핀, GND, 4.7 kΩ pull-up, 방수형 센서의 실제 선 색상 사양을 확인합니다. |
| DHT11이 NaN | DATA/VCC/GND, 모듈 pull-up, 2.5초 읽기 간격을 확인합니다. 배선을 짧게 하고 3V3 전압 강하를 측정합니다. |
| GUVA ADC가 0 또는 최대값 고정 | OUT/VCC/GND 핀 순서, 출력전압, 단선·단락을 확인합니다. 4095 근처 전압이면 즉시 분리해 3.3 V 초과 여부를 측정합니다. |
| 조도센서 ADC가 이상함 | `OUT`이 아날로그 출력인지 확인합니다. 밝음/가림에서 Raw가 변하는지 보고, 반대 방향이면 LIGHT 정규화 양 끝값을 역순으로 넣습니다. |
| 모터가 돌지 않음 | 외부 5 V와 공통 GND, ULN 전원, 모터 커넥터, `STATUS` 상태를 확인합니다. ERROR/STOP 후에는 HOME이 필요합니다. |
| 모터가 떨기만 함 | IN1~IN4 결선 순서 또는 모터 코일 순서 문제일 가능성이 큽니다. 표대로 복원하고 속도/가속도를 낮춥니다. |
| 모터가 반대로 회전 | 즉시 STOP 후 `MOTOR_DIRECTION_INVERT`만 반전해 재업로드하고 HOME부터 다시 시험합니다. |
| 모터 토크 부족 | 5 V 전원의 실제 전압/전류, 축 마찰, 장력, 기어 걸림을 확인하고 속도·가속도를 낮춥니다. 정격 이상 전압을 올리지 마십시오. |
| ULN2003 LED만 켜지고 모터가 안 움직임 | 모터 5핀 플러그 접촉, 외부 전원 용량, 모터 단선, 보드 출력 소켓을 확인합니다. LED 점등만으로 코일 전류가 흐른다고 단정할 수 없습니다. |
| Upper Endstop을 눌러도 멈추지 않음 | 즉시 전원을 끄고 `ENDSTOPS`로 논리값을 확인합니다. S/GND, Active LOW와 D4를 확인합니다. D5는 사용하지 않습니다. |
| Homing 실패 | 모터 감김 방향, Upper 신호, 기계 걸림, 90초/14,000스텝 한계를 확인합니다. 한계를 늘리기 전에 실제 이동이 정상인지 먼저 확인합니다. |
| Calibration 실패 | `MEASUREMENT READY` 이후 JOG를 완료했는지, 측정값이 300~12,000스텝인지, 정지 상태에서 `CAL SET`을 입력했는지 확인합니다. |
| POS 위치가 실제 위치와 맞지 않음 | `CAL CLEAR`→`CALIBRATE`를 다시 수행하고 줄 감김 직경 변화, 벨트/축 미끄러짐, 비선형 기구를 점검합니다. 퍼센트는 스텝 거리에 선형 매핑됩니다. |
| 반복 후 위치 오차 누적 | 탈조·미끄러짐을 줄이도록 속도/가속도와 부하를 낮춥니다. 주기적으로 HOME을 수행하고, 심하면 엔코더나 중간 위치 센서가 필요합니다. |
| EEI가 너무 민감함 | 정규화 범위를 실측값으로 넓히고 불안정 센서 weight를 낮춥니다. EMA, 히스테리시스, 안정시간을 늘립니다. |
| 차광막이 계속 위아래로 움직임 | `EEI_HYSTERESIS`, `CONTROL_STABLE_TIME_MS`, `MIN_POSITION_CHANGE_PERCENT`를 늘리고 센서가 햇빛/차광막 그림자에 의해 자기 피드백을 만드는 위치인지 확인합니다. |

### 참고한 공식 자료

- [Arduino Nano ESP32 데이터시트](https://docs.arduino.cc/resources/datasheets/ABX00083-datasheet.pdf)
- [TI ULN2003A 제품 및 데이터시트](https://www.ti.com/product/ULN2003A)
- [Analog Devices DS18B20 pull-up 안내](https://support.analog.com/en-US/knowledgebase/article/000094969)
- [Melexis MLX90614 데이터시트](https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90614)
- [AccelStepper 공식 문서](https://www.airspayce.com/mikem/arduino/AccelStepper/)
