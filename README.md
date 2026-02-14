# CAN Bus Discover

PT-CAN 시그널 자동 탐색 전용 도구. BLE/Classic BT OBD 어댑터를 패시브 모드로 연결하여 6단계 캡처를 수행하고, 모터스포츠에 필요한 CAN 시그널(스티어링, RPM, 스로틀, 브레이크, 기어, 휠스피드)을 자동 식별한다.

## 빌드

```bash
cd tests/canbus_discover
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
Phase 1: Baseline    (5초)  — 아무 조작 없이 기본 트래픽 캡처
Phase 2: Steering    (5초)  — 스티어링 휠 좌/우 반복
Phase 3: Throttle    (8초)  — 액셀 페달 밟기/떼기 2-3회
Phase 4: Brake       (8초)  — 브레이크 페달 밟기/떼기 2-3회
Phase 5: Gear        (10초) — P → R → N → D 시프트
Phase 6: Wheel Speed (15초) — 저속 주행 (30+ km/h), 가능하면 커브 포함
    │
분석: 각 Phase의 CAN ID별 바이트 변화량을 Baseline과 비교
    │
JSON 출력 (stdout)
```

각 Phase 시작 전 Enter 키 입력을 기다린다. 사용자가 해당 조작을 준비한 후 Enter를 누르면 캡처가 시작된다.

## 출력 예시

```json
{
  "steering":{"can_id":"0x0A5","dlc":8,"hz":100.0,"byte":2,"byte2":3,"score":48.5},
  "rpm":{"can_id":"0x316","dlc":8,"hz":50.0,"byte":2,"byte2":3,"score":35.2},
  "throttle":{"can_id":"0x1A0","dlc":8,"hz":50.0,"byte":5,"score":28.0},
  "brake":{"can_id":"0x1A0","dlc":8,"hz":50.0,"byte":4,"score":22.0},
  "gear":{"can_id":"0x3B0","dlc":8,"hz":10.0,"byte":0,"score":15.0},
  "wheel_speed":{"can_id":"0x1A6","dlc":8,"hz":50.0,"byte":0,"byte2":1,"score":42.0}
}
```

stderr에는 사람이 읽을 수 있는 요약이 출력된다:

```
=== Results ===
  steering    0x0A5 [8] @ 100.0 Hz  byte 2,3  (score 48.5)
  rpm         0x316 [8] @ 50.0 Hz  byte 2,3  (score 35.2)
  throttle    0x1A0 [8] @ 50.0 Hz  byte 5  (score 28.0)
  brake       0x1A0 [8] @ 50.0 Hz  byte 4  (score 22.0)
  gear        0x3B0 [8] @ 10.0 Hz  byte 0  (score 15.0)
  wheel_speed 0x1A6 [8] @ 50.0 Hz  byte 0,1  (score 42.0)
```

## 분석 알고리즘

각 CAN ID의 바이트별로 baseline 대비 변화량을 점수화한다:

```
change_score = max(0, active_range - baseline_range)
             + max(0, active_unique - baseline_unique) * 0.5
```

- `range`: 해당 바이트의 (max - min) 값
- `unique`: 관측된 고유 값 수

Phase별로 가장 높은 점수의 CAN ID를 해당 시그널로 분류하며, 이미 식별된 CAN ID는 후순위 Phase에서 제외한다.

## 주의사항

- macOS Bluetooth 권한 필요 (시스템 설정 > 개인정보 보호 및 보안)
- **PT-CAN 직결 상태**에서만 의미 있음. D-CAN(OBD 포트)에서는 브로드캐스트가 없거나 제한적
- 어댑터는 항상 Silent Mode(`ATCSM1`)로 동작 — CAN 버스에 ACK를 보내지 않음
- **IGN ON 상태**에서 실행해야 함 (시동 OFF 시 PT-CAN도 조용)
- Phase당 최대 4096 프레임 캡처
