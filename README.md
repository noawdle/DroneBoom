# DroneBoom
대드론

# UAV 기반 GNSS 스푸핑 영향 범위 탐지 및 공간 정보 시각화 시스템

## 1. 프로젝트 개요

본 프로젝트는 UAV가 이동하면서 수집한 GNSS 위치정보의 이상을 탐지하고, 해당 이상이 발생한 위치를 공간적으로 시각화하는 시스템을 구현하는 것을 목표로 한다.

현재 단계에서는 실제 UAV 비행 및 RF 기반 GNSS Spoofing 대신 **PX4 SITL + Gazebo Harmonic 기반 시뮬레이션 환경**을 사용한다.

Gazebo의 X500 UAV가 일정한 2차원 공간을 비행하면서 GNSS, IMU, PX4 Local Position 및 Gazebo Ground Truth 데이터를 수집하고, 특정 공간에 소프트웨어 기반 GNSS 위치 이상을 주입한다.

이후 GNSS 위치와 독립 기준 위치의 차이인 Residual을 계산하여 다음 상태로 분류한다.

- `NORMAL`
- `OBSERVE`
- `SPOOFING_SUSPECTED`

탐지 결과는 Streamlit Dashboard를 통해 UAV 이동 경로, 위치 오차, 탐지 상태 및 공간별 GNSS 이상 분포로 시각화한다.

> 현재 구현된 Spoofing은 실제 RF 신호를 송신하는 방식이 아니라, 정상 GNSS 데이터에 소프트웨어적으로 위치 이상을 추가하는 시뮬레이션 방식이다.

---

## 2. 전체 시스템 구성

```text
Gazebo X500 UAV
      │
      ▼
   PX4 SITL
      │
      │ MAVLink
      ▼
Python Data Logger
      │
      ├── GNSS
      ├── IMU
      ├── PX4 Local Position
      └── Gazebo Ground Truth
      │
      ▼
     CSV
      │
      ▼
GNSS Spoofing Scenario Injection
      │
      ▼
GNSS ↔ Ground Truth Residual 계산
      │
      ▼
NORMAL / OBSERVE / SPOOFING_SUSPECTED
      │
      ▼
Streamlit Spatial Dashboard
```

---

## 3. 개발 환경

| 항목 | 환경 |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| Architecture | x86_64 |
| PX4 | v1.17.0 |
| Simulator | Gazebo Harmonic |
| Gazebo Sim | gz-sim 8.15.0 |
| UAV Model | X500 Multicopter |
| Language | Python |
| Communication | MAVLink |
| Visualization | Streamlit + Plotly |

---

## 4. PX4 SITL + Gazebo X500 환경 구축

PX4 SITL과 Gazebo Harmonic을 이용하여 X500 멀티콥터 시뮬레이션 환경을 구축하였다.

기본 실행 명령:

```bash
cd /home/leedaeun/DroneBoom/DroneBoom
./run-px4-x500.sh
```

실행 구조:

```text
PX4 SITL
   ↕
Gazebo X500
   ↕
MAVLink
   ↕
Python Programs
```

Gazebo 내부에는 `x500_0` 모델이 생성되며 PX4 SITL과 연결된다.

---

## 5. 기본 자동 비행

기본 비행 검증을 위해 다음 파일을 구현하였다.

```text
test-px4-flight.py
```

비행 순서:

```text
Arm
 ↓
Takeoff
 ↓
약 3 m 이동
 ↓
Landing
 ↓
Disarm
```

초기 테스트에서는 MAVLink GCS Heartbeat 손실로 인해 Data Link Failsafe가 발생했으나, 비행 중 Heartbeat를 지속적으로 송신하도록 수정하여 정상 동작을 확인하였다.

최종 결과:

```text
FLIGHT_TEST_PASS
```

---

## 6. Lawnmower 방식 2D 영역 스캔

단순 직선 비행만으로는 공간 분포를 시각화하기 어렵기 때문에 일정 면적 전체를 스캔하는 Lawnmower 방식의 자동 비행을 구현하였다.

사용 파일:

```text
test-px4-lawnmower.py
```

현재 비행 설정:

```text
Scan Area    : 약 8 m × 8 m
Altitude     : 2.5 m
Lane Spacing : 1.0 m
Scan Lines   : 9
Flight Speed : 약 1.0 m/s
```

비행 형태:

```text
→ → → → → → → →
              ↓
← ← ← ← ← ← ← ←
↓
→ → → → → → → →
              ↓
← ← ← ← ← ← ← ←
```

실제 검증 결과:

```text
LAWNMOWER_FLIGHT_PASS
```

Gazebo Ground Truth 기준 실제 스캔 범위:

```text
East  : 약 8.18 m
North : 약 8.39 m
```

비행 중 Data Link Failsafe는 발생하지 않았으며 Landing 후 정상 Disarm이 확인되었다.

---

## 7. 비행 데이터 수집

비행 중 센서 및 위치정보를 저장하기 위해 다음 데이터 로거를 구현하였다.

```text
collect-flight-data.py
```

약 10 Hz 주기로 다음 데이터를 CSV 파일에 저장한다.

### GNSS

```text
gps_lat
gps_lon
gps_alt
```

### PX4 Local Position

```text
local_x
local_y
local_z
```

### Gazebo Ground Truth

```text
true_x
true_y
true_z
```

### Attitude

```text
roll
pitch
yaw
```

### IMU

```text
accel_x
accel_y
accel_z

gyro_x
gyro_y
gyro_z
```

### Timestamp

```text
timestamp
```

CSV는 다음 형식으로 자동 생성된다.

```text
data/flight_YYYYMMDD_HHMMSS.csv
```

---

## 8. Ground Truth와 UWB 역할

현재 시뮬레이션에서는 실제 UWB Anchor / Tag 모델을 구현하지 않고 **Gazebo Ground Truth를 독립 기준 위치로 사용**한다.

현재 구조:

```text
GNSS
= 이상 여부를 검사하는 위치정보

Gazebo Ground Truth
= UAV의 기준 위치
= 향후 실제 시스템에서 UWB가 담당할 역할
```

따라서 현재 Gazebo Ground Truth는 실제 UWB 측정값 자체가 아니라 **UWB 기반 독립 기준 위치를 대체하는 값**이다.

실제 하드웨어 구현 단계에서는 Gazebo Ground Truth 부분을 실제 UWB 위치 데이터로 교체할 수 있다.

---

## 9. GNSS 좌표 변환

GNSS 데이터는 위도 / 경도 / 고도 기반 좌표이고 Gazebo Ground Truth는 meter 기반 Cartesian 좌표이므로 직접 비교할 수 없다.

이를 위해 GNSS 좌표를 다음 과정으로 변환한다.

```text
WGS84 Geodetic
      ↓
     ECEF
      ↓
 Local ENU
```

최종적으로 다음 좌표를 생성한다.

```text
gps_east
gps_north
gps_up

true_east
true_north
true_up
```

Gazebo Ground Truth는 시작 위치를 기준으로 상대 좌표를 생성한다.

```text
true_east  = true_x - initial_true_x
true_north = true_y - initial_true_y
true_up    = true_z - initial_true_z
```

정상 비행 데이터에서 GNSS와 Ground Truth 사이의 수평 위치 오차는 다음과 같이 확인되었다.

```text
Mean   : 약 0.0401 m
Median : 약 0.0393 m
Max    : 약 0.1266 m
P95    : 약 0.0676 m
```

---

## 10. Software-Based GNSS Spoofing Scenario

본 프로젝트에서는 실제 RF GNSS Spoofing 신호를 생성하지 않는다.

대신 정상 GNSS 데이터의 복사본에 위치 이상을 소프트웨어적으로 추가한다.

```text
Original GNSS
      │
      ├─────────────── 원본 보존
      │
      ▼
Spoofed GNSS Copy
      │
      ▼
Anomaly Detection
```

원본 GNSS 및 Gazebo Ground Truth 데이터는 변경하지 않는다.

스푸핑이 적용된 데이터는 별도 컬럼으로 저장한다.

```text
spoofed_gps_east
spoofed_gps_north
spoofed_gps_up
```

---

## 11. 단일 GNSS Spoofing Scenario

초기 알고리즘 검증을 위해 다음 프로그램을 구현하였다.

```text
inject-gnss-spoofing.py
```

특정 공간에서 GNSS에 다음 위치 Offset을 적용하였다.

```text
East  : +3.0 m
North : +2.0 m
```

정상 GNSS-Ground Truth 평균 수평 오차는 약 4 cm 수준이었으며, Spoofing 주입 이후 약 3.6 m 수준으로 증가하였다.

---

## 12. Random Spatial Spoofing Zones

공간적인 GNSS 이상 분포를 표현하기 위해 다음 프로그램을 구현하였다.

```text
inject-random-spoofing-zones.py
```

UAV가 스캔하는 2차원 영역 내부에 여러 개의 타원형 GNSS 이상 영역을 생성한다.

각 영역은 다음 정보를 가진다.

```text
center_east
center_north

radius_east
radius_north

GNSS offset east
GNSS offset north
```

실험 재현성을 위해 Random Seed를 사용한다.

기본 설정:

```text
Seed       : 42
Zone Count : 4
```

Seed 42 기준 생성된 Zone:

| Zone | Center East/North [m] | Radius East/North [m] | GNSS Offset East/North [m] |
|---|---|---|---|
| 1 | 2.32 / 5.06 | 0.68 / 0.88 | +1.87 / +2.66 |
| 2 | 7.24 / 2.36 | 0.66 / 0.88 | +2.70 / -2.95 |
| 3 | 6.32 / 5.04 | 0.77 / 0.99 | -2.21 / +2.97 |
| 4 | 0.84 / 1.40 | 0.63 / 0.71 | -1.41 / +3.64 |

각 Zone은 실제 UAV 스캔 영역 내부에 위치하며 Lawnmower 경로가 해당 영역을 통과하도록 구성되어 있다.

Random Seed를 변경하면 다른 공간 배치를 생성할 수 있다.

예:

```bash
./inject-random-spoofing-zones.py data/flight_20261013_231607.csv --seed 123
```

---

## 13. GNSS 이상 탐지 알고리즘

탐지 프로그램:

```text
detect-gnss-anomaly.py
```

는 Spoofing Zone의 위치나 GNSS Offset 정보를 사용하지 않는다.

탐지기는 오직 GNSS 위치와 기준 위치 사이의 Residual을 이용한다.

수평 위치 Residual:

```text
horizontal_residual
=
sqrt(
    (GNSS_East - True_East)^2
    +
    (GNSS_North - True_North)^2
)
```

탐지 상태는 다음과 같이 구성된다.

```text
NORMAL
OBSERVE
SPOOFING_SUSPECTED
```

순간적인 센서 오차 하나만으로 Spoofing으로 판단하지 않도록 연속 샘플 기반 Persistence 조건을 적용하였다.

현재 기본 Persistence:

```text
3 consecutive samples
```

---

## 14. Threshold 설정

정상 GNSS-Ground Truth Residual 통계를 기반으로 Threshold를 계산한다.

예:

```text
Warning Threshold
= Normal Mean + 3 × Standard Deviation
```

Spoofing 의심 Threshold 역시 정상 데이터 통계를 기반으로 결정한다.

즉 고정된 임의 숫자가 아니라 정상 데이터 분포를 먼저 분석한 뒤 탐지 기준을 설정한다.

---

## 15. 초기 탐지 성능

초기 단일 Spoofing Zone 실험 결과:

```text
TP = 9
FP = 0
TN = 644
FN = 4
```

성능:

```text
Precision = 1.0000
Recall    = 0.6923
Accuracy  = 0.9939
```

일부 False Negative는 Persistence 조건 때문에 Spoofing Zone에 진입한 직후 일부 샘플이 `OBSERVE` 상태로 유지되면서 발생하였다.

---

## 16. Random Spatial Zone 탐지 성능

8 m × 8 m Lawnmower 비행과 4개의 Random Spoofing Zone을 적용한 결과:

```text
TP = 84
FP = 0
TN = 1372
FN = 14
```

성능:

```text
Precision = 1.0000
Recall    = 0.8571
Accuracy  = 0.9905
```

Zone별 탐지 결과:

```text
Zone 1 : 15 / 17 samples detected
Zone 2 : 22 / 26 samples detected
Zone 3 : 24 / 28 samples detected
Zone 4 : 23 / 27 samples detected
```

정상 영역을 Spoofing 영역으로 잘못 판단한 False Positive는 현재 실험에서 발생하지 않았다.

---

## 17. Streamlit Dashboard

탐지 결과를 시각화하기 위해 다음 Dashboard를 구현하였다.

```text
dashboard.py
```

Dashboard에서 다음 정보를 확인할 수 있다.

### 주요 KPI

```text
Total Samples
Maximum Residual
Spoofing Detection Count
Normal Mean Residual
Precision
Recall
Accuracy
```

### Visualization

```text
UAV Flight Path
GNSS vs Ground Truth
Residual vs Time
Detection Timeline
Spatial Residual Map
Spatial Grid Map
Confusion Matrix
Detection Result Table
```

Dashboard 실행:

```bash
cd /home/leedaeun/DroneBoom/DroneBoom
python3 -m streamlit run dashboard.py
```

---

## 18. Spatial Grid Map

UAV가 실제로 이동한 공간을 일정 크기의 Grid Cell로 나누고 각 Cell에서 측정된 GNSS Residual을 이용하여 공간적인 이상 분포를 표현한다.

현재 지원 Cell Size:

```text
0.5 m
1.0 m
```

개념적인 상태 표현:

```text
Green  : 낮은 GNSS 위치 이상
Yellow : 관찰 상태
Red    : 높은 GNSS 위치 이상 / Spoofing 의심
Gray   : No Data
```

UAV가 실제로 방문하지 않은 공간에는 데이터를 임의로 생성하지 않는다.

즉 측정되지 않은 위치는 `No Data`로 처리한다.

따라서 현재 Spatial Map의 정확한 의미는 다음과 같다.

> UAV가 실제로 스캔한 공간에서 관측된 GNSS 위치 이상 분포

이는 실제 RF 전파 Coverage 자체를 의미하지 않는다.

---

## 19. Scenario Ground Truth Overlay

Dashboard에서는 실험용으로 생성한 실제 Spoofing Zone과 탐지 결과를 비교할 수 있다.

```text
Show Scenario Ground Truth
```

기능을 활성화하면 생성된 타원형 Scenario Zone을 지도 위에 표시할 수 있다.

이를 통해 다음을 비교할 수 있다.

```text
실제로 설정한 GNSS 이상 영역
            VS
탐지 알고리즘이 판단한 이상 영역
```

---

## 20. Confusion Matrix

탐지 알고리즘의 성능은 Confusion Matrix를 이용하여 평가한다.

```text
TP
= 실제 Spoofing이며 탐지 성공

FP
= 정상인데 Spoofing으로 오탐

TN
= 정상이며 정상으로 판단

FN
= 실제 Spoofing이지만 탐지하지 못함
```

이를 이용하여 다음 성능지표를 계산한다.

```text
Precision
Recall
Accuracy
```

---

## 21. 주요 파일 구조

```text
DroneBoom/
│
├── run-px4-x500.sh
├── px4-gz-env.sh
│
├── test-px4-flight.py
├── test-px4-lawnmower.py
│
├── collect-flight-data.py
│
├── inject-gnss-spoofing.py
├── inject-random-spoofing-zones.py
│
├── detect-gnss-anomaly.py
│
├── dashboard.py
│
├── PX4-Autopilot/
│
└── data/
    ├── flight_*.csv
    │
    └── processed/
        ├── spoofed_*.csv
        ├── detected_*.csv
        ├── spoofed_random_*.csv
        ├── detected_random_*.csv
        └── scenario_random_*.json
```

---

# 22. 전체 실행 방법

## Terminal 1 - PX4 + Gazebo 실행

```bash
cd /home/leedaeun/DroneBoom/DroneBoom
./run-px4-x500.sh
```

---

## Terminal 2 - 데이터 수집

```bash
cd /home/leedaeun/DroneBoom/DroneBoom
source ./px4-gz-env.sh
./collect-flight-data.py
```

---

## Terminal 3 - Lawnmower 자동 비행

```bash
cd /home/leedaeun/DroneBoom/DroneBoom
source ./px4-gz-env.sh
./test-px4-lawnmower.py
```

비행 완료 후 데이터 로거에서 `Ctrl + C`를 눌러 CSV를 저장한다.

생성 파일 예:

```text
data/flight_20261013_231607.csv
```

---

## Random Spoofing Zone 생성

```bash
cd /home/leedaeun/DroneBoom/DroneBoom

./inject-random-spoofing-zones.py \
data/flight_20261013_231607.csv \
--seed 42
```

생성 파일:

```text
data/processed/spoofed_random_flight_20261013_231607.csv
```

Scenario 정보:

```text
data/processed/scenario_random_flight_20261013_231607.json
```

---

## GNSS 이상 탐지

```bash
./detect-gnss-anomaly.py \
data/processed/spoofed_random_flight_20261013_231607.csv
```

생성 파일:

```text
data/processed/detected_random_flight_20261013_231607.csv
```

---

## Dashboard 실행

```bash
python3 -m streamlit run dashboard.py
```

브라우저에서 Streamlit Dashboard를 열고 다음 파일을 선택한다.

```text
detected_random_flight_20261013_231607.csv
```

---

## 23. 현재 구현 상태

현재까지 구현된 기능:

```text
PX4 SITL + Gazebo X500                ✅
MAVLink 통신                          ✅
자동 Arm / Takeoff / Landing          ✅
기본 자동 비행                        ✅
Lawnmower 2D Area Scan               ✅
GNSS 데이터 수집                      ✅
IMU 데이터 수집                       ✅
PX4 Local Position 수집               ✅
Gazebo Ground Truth 수집              ✅
CSV Data Logger                      ✅
WGS84 → Local ENU 변환                ✅
Software GNSS Spoofing Injection      ✅
Random Spatial Spoofing Zones         ✅
GNSS-Ground Truth Residual Detection  ✅
Persistence-based Detection           ✅
Precision / Recall / Accuracy 평가    ✅
Confusion Matrix                     ✅
Spatial Grid Map                     ✅
Scenario Ground Truth Overlay         ✅
Streamlit Dashboard                  ✅
```

---

## 24. 향후 개발 계획

향후 다음 기능을 추가할 예정이다.

- Jetson Orin Nano Companion Computer 연동
- 실제 UAV Flight Controller 연동
- 실제 GNSS Receiver 연동
- 실제 UWB Anchor / Tag 적용
- UWB 측정 Noise 반영
- UWB NLOS 환경 분석
- IMU 기반 추가 이상 탐지
- GNSS Drift / Ramp Spoofing Scenario
- 실시간 데이터 수집 및 탐지
- 실시간 Dashboard
- 반복 실험 기반 성능 통계
- 실제 UAV 비행 시험

---

## 25. 프로젝트 해석 시 주의사항

현재 구현된 GNSS Spoofing은 실제 RF 신호를 송신하는 방식이 아니다.

모든 Spoofing Scenario는 정상 GNSS 데이터의 복사본에 소프트웨어적으로 위치 이상을 주입하는 방식으로 수행한다.

또한 현재 시뮬레이션에서는 실제 UWB 센서 대신 Gazebo Ground Truth를 독립 기준 위치로 사용한다.

현재 Spatial Map은 실제 RF 전파 범위를 직접 측정한 것이 아니라:

> UAV가 실제로 이동하며 측정한 위치에서 관측된 GNSS 이상 분포

를 의미한다.

따라서 본 시스템은 현재 단계에서 **GNSS 이상 탐지 및 공간 정보 시각화 알고리즘의 시뮬레이션 기반 검증 시스템**으로 정의한다.

---

## 26. 최종 프로젝트 흐름

```text
PX4 SITL + Gazebo
        ↓
X500 UAV Lawnmower Flight
        ↓
GNSS + IMU + Ground Truth 수집
        ↓
CSV 저장
        ↓
Random Spatial GNSS Anomaly 생성
        ↓
Spoofed GNSS 생성
        ↓
GNSS ↔ Ground Truth Residual 계산
        ↓
NORMAL
OBSERVE
SPOOFING_SUSPECTED
        ↓
Precision / Recall / Accuracy 평가
        ↓
2D Spatial Grid Map
        ↓
Streamlit Dashboard
```

---

## Project Status

**Simulation-based prototype in development**

현재 PX4 SITL + Gazebo 기반 UAV 자동 비행, 데이터 수집, GNSS 이상 주입, 탐지 알고리즘 및 공간 시각화 Dashboard까지 구현 완료하였다.