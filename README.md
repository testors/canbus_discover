# CAN Bus Discover

PT-CAN 시그널 자동 탐색 도구. BLE/Classic BT OBD 어댑터를 패시브 모드로 연결하여 대화형 6단계 캡처를 수행하고, 모터스포츠에 필요한 CAN 시그널(스티어링, RPM, 스로틀, 브레이크, 기어, 휠스피드)을 자동 식별한다.

## 빌드

```bash
mkdir -p build && cd build && cmake .. && make
```

또는:

```bash
./build.sh
```

## 사용법

```bash
# 특정 어댑터 지정
./build/canbus_discover vLinker

# 첫 번째 발견 디바이스에 자동 연결
./build/canbus_discover

# JSON만 파싱 (로그 숨김)
./build/canbus_discover vLinker 2>/dev/null
```

- `device_prefix`: BLE 이름 prefix 매칭 (대소문자 무시). 생략 시 첫 번째 디바이스에 자동 연결.
- JSON 출력은 stdout, 로그는 stderr.

## 동작 흐름

```
BLE/Classic BT 스캔 → 어댑터 연결 → PT-CAN 패시브 모드 초기화
    │
    ├── ATSP6    (CAN 11-bit 500k 강제)
    ├── ATCSM1   (Silent Mode — CAN ACK 전송 안 함)
    ├── ATCAF0   (Raw CAN 포맷)
    └── ATH1     (헤더 표시)
    │
Phase 1: Baseline     (5초)  — 아무 조작 없이 기본 트래픽 캡처
Phase 2: Steering     (8초)  — 스티어링 풀 좌/우 (lock to lock) 반복
Phase 3: Throttle     (8초)  — 액셀 페달 끝까지 밟기/떼기 반복
Phase 4: Brake        (8초)  — 브레이크 페달 끝까지 밟기/떼기 반복
Phase 5: Gear        (10초)  — P → R → N → D 시프트 [스킵 가능]
Phase 6: Wheel Speed (15초)  — 저속 주행 30+ km/h [스킵 가능]
    │
분석: Phase별로 CAN ID 바이트 변화량을 Baseline 대비 비교
    + 시그널 특성화 (엔디언, 부호, raw 범위 추정)
    │
사용자 확인: Phase별 결과에 대해 포함/제외 선택
    │
JSON 출력 (stdout)
```

### 대화형 흐름

각 Phase는 대화형으로 진행된다:

1. **Phase 시작 전**: 조작 안내 표시. Gear/Wheel Speed Phase는 추가 경고 표시.
   - `[Enter]` 진행 / `[s]` 스킵

2. **캡처 완료 후**: 감지된 시그널의 CAN ID, 주파수, 바이트 위치, 신뢰도, 데이터 타입을 표시.
   - `[y]` 결과에 포함 / `[n]` 제외

3. Throttle Phase에서는 RPM도 함께 감지 — 각각 개별 확인.

**Phase 5 (Gear) 경고**: 수동 변속기 차량은 기어 포지션을 CAN으로 브로드캐스트하지 않을 수 있음.

**Phase 6 (Wheel Speed) 경고**: ABS/ESC가 별도 CAN 버스에 있을 수 있으며, 실제 주행이 필요.

## 시그널 특성화

각 시그널 감지 시 데이터 타입도 함께 분석한다. 정확한 특성화를 위해 캡처 중 **풀 레인지 입력** (최대/최소까지 조작)을 권장한다.

| 속성 | 설명 | 판별 방법 |
|------|------|----------|
| **Endianness** | Big-endian / Little-endian | 2바이트 시그널에서 range가 작은 바이트가 MSB. MSB가 낮은 주소면 big-endian |
| **Signedness** | Signed / Unsigned | MSB의 seen[] 비트맵이 0x00 근처와 0xFF 근처에 bimodal 분포면 signed |
| **Raw Range** | 관측된 최솟값/최댓값 | `(MSB<<8)\|LSB` 조합으로 계산. Signed면 int16_t 해석 |
| **Confidence** | HIGH / MEDIUM / LOW | 시그널별 패턴 매칭 품질에 따라 결정 |

### 신뢰도 기준

| 신뢰도 | 의미 |
|--------|------|
| **HIGH** | 시그널 고유 패턴과 일치 (예: 스티어링 = 2바이트 + 고주파 + 최고 점수) |
| **MEDIUM** | 보조 패턴과 일치 (예: 휠스피드 = 2+바이트 변화 + 30Hz 이상) |
| **LOW** | 패턴 매칭 없이 최고 점수 후보만 반환 (오탐 가능성 있음) |

## 출력 예시

### stderr (사용자용 요약)

```
=== Phase 2/6: Steering ===
  Turn steering wheel to FULL LEFT, then FULL RIGHT (lock to lock). Repeat 2-3 times.
  [Enter] proceed  [s] skip >
  Capturing 8 seconds... 842 frames, 23 CAN IDs
  Signal detected:
    steering     0x0A5 [8] @ 100.0 Hz  byte 2,3  score=48.5  confidence=HIGH
                 big-endian signed  raw=[-5420..5380]
  Include steering? [y/n] > y

...

=== Final Results ===
    steering     0x0A5 [8] @ 100.0 Hz  byte 2,3  score=48.5  confidence=HIGH
                 big-endian signed  raw=[-5420..5380]
    rpm          0x316 [8] @ 50.0 Hz  byte 2,3  score=35.2  confidence=HIGH
                 big-endian unsigned  raw=[780..6200]
    throttle     0x1A0 [8] @ 50.0 Hz  byte 5  score=28.0  confidence=HIGH
                 unsigned  raw=[0..255]
    brake        0x1A0 [8] @ 50.0 Hz  byte 4  score=22.0  confidence=HIGH
                 unsigned  raw=[0..180]
    wheel_speed  0x1A6 [8] @ 50.0 Hz  byte 0,1  score=42.0  confidence=HIGH
                 big-endian unsigned  raw=[0..892]
```

### stdout (JSON)

```json
{
  "steering":{"can_id":"0x0A5","dlc":8,"hz":100.0,"byte":2,"byte2":3,"score":48.5,"confidence":"HIGH","endian":"big","signed":true,"raw_min":-5420,"raw_max":5380},
  "rpm":{"can_id":"0x316","dlc":8,"hz":50.0,"byte":2,"byte2":3,"score":35.2,"confidence":"HIGH","endian":"big","signed":false,"raw_min":780,"raw_max":6200},
  "throttle":{"can_id":"0x1A0","dlc":8,"hz":50.0,"byte":5,"score":28.0,"confidence":"HIGH","signed":false,"raw_min":0,"raw_max":255},
  "brake":{"can_id":"0x1A0","dlc":8,"hz":50.0,"byte":4,"score":22.0,"confidence":"HIGH","signed":false,"raw_min":0,"raw_max":180},
  "wheel_speed":{"can_id":"0x1A6","dlc":8,"hz":50.0,"byte":0,"byte2":1,"score":42.0,"confidence":"HIGH","endian":"big","signed":false,"raw_min":0,"raw_max":892}
}
```

- `confidence`: `"HIGH"`, `"MEDIUM"`, `"LOW"`
- `endian`: `"big"` 또는 `"little"` (2바이트 시그널만)
- `signed`: `true` 또는 `false`
- `raw_min`, `raw_max`: 캡처 중 관측된 원시 값 범위

## 분석 알고리즘

각 CAN ID의 바이트별로 baseline 대비 변화량을 점수화한다:

```
change_score = max(0, active_range - baseline_range)
             + max(0, active_unique - baseline_unique) * 0.5
```

- `range`: 해당 바이트의 (max - min) 값
- `unique`: 관측된 고유 값 수

### 시그널별 분류 전략

| 시그널 | 1순위 패턴 | 2순위 패턴 |
|--------|-----------|-----------|
| **Steering** | 2바이트 변화 + 최고 Hz + 최고 점수 | 최고 점수 |
| **RPM** | Throttle Phase에서 2바이트 변화 + Hz ≥ 40 | 최고 점수 |
| **Throttle** | Throttle Phase에서 RPM과 다른 최고 점수 | — |
| **Brake** | 1바이트 변화 + Hz ≥ 30 | 최고 점수 |
| **Gear** | 1바이트 + unique ≤ 8 | 최고 점수 |
| **Wheel Speed** | 4+ 바이트 변화 + Hz ≥ 30 | 2+ 바이트 변화 + Hz ≥ 30 |

이미 식별된 CAN ID는 후순위 Phase에서 자동 제외된다.

## 주의사항

- macOS Bluetooth 권한 필요 (시스템 설정 > 개인정보 보호 및 보안)
- **PT-CAN 직결 상태**에서만 의미 있음. D-CAN(OBD 포트)에서는 브로드캐스트가 없거나 제한적
- 어댑터는 항상 Silent Mode(`ATCSM1`)로 동작 — CAN 버스에 ACK를 보내지 않음
- **IGN ON 상태**에서 실행해야 함 (시동 OFF 시 PT-CAN도 조용)
- Phase당 최대 4096 프레임 캡처
- 특성화 정확도는 캡처 중 입력 범위에 의존 — 풀 레인지 조작 권장
