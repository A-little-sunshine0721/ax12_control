# ax12_control

ROS 2 기반 **Dynamixel 다관절 제어 패키지**입니다.
AX-12A(Protocol 1.0)와 XL330(Protocol 2.0)을 **하나의 버스에서 함께** 제어하고,
두 모터를 하나의 관절처럼 묶는 **동기 제어**, **그리퍼 전류 제한 제어**, **키보드 조작**, **실시간 상태 그래프**를 제공합니다.

## 주요 기능

- **통합 제어기 (`arm_controller`)**: 연결된 모터를 자동 인식(AX-12A / XL330)해서 관절로 등록하고 50 Hz로 제어
- **2축 동기 제어**: ID 2·3을 반대 방향으로 묶어 같은 각도만큼 동시에 구동 (교차 결합 PI)
- **그리퍼 모드**: XL330을 전류 제한 위치 제어(모드 5)로 구동해 물체를 잡은 채 토크 유지
- **안전 기능**: 범위 밖 명령 거부, 과부하·과열·통신 실패 시 보호, anti-windup
- **키보드 조작 (`key_teleop`)**: ID 선택 후 화살표로 이동, 원위치 저장·복귀
- **실시간 그래프 (`monitor_plot`)**: 위치·오차·부하·전류·온도·전압

## 하드웨어 구성

```
Ubuntu PC ──USB──► OpenRB-150 (usb_to_dynamixel 펌웨어) ──TTL 버스 (1 Mbps)──┬─ AX-12A × 5  (12V)
                       ▲ 12V (VIN 터미널, 점퍼 VIN(DXL))                     └─ XL330 × 1  (5V, 컨버터)
```

| ID | 모델 | 프로토콜 | 관절 | 비고 |
|---|---|---|---|---|
| 1 | AX-12A | 1.0 | joint1 | 단독 |
| 2, 3 | AX-12A | 1.0 | joint23 | **동기 묶음** (ID 3은 반대 방향) |
| 4 | AX-12A | 1.0 | joint4 | 단독 |
| 5 | AX-12A | 1.0 | joint5 | 단독 |
| 6 | XL330-M077 | 2.0 | joint6 | **그리퍼** |

- 모든 모터는 **1 Mbps**, ID는 서로 겹치지 않아야 합니다.
- TTL 3핀 커넥터: **1 GND / 2 VDD(+, 가운데) / 3 DATA**
- ⚠ XL330은 **5V** 모터입니다. 12V가 들어가면 고장납니다.

## 소프트웨어 요구사항

- Ubuntu 24.04, ROS 2 Jazzy
- `ros-jazzy-dynamixel-sdk`, `python3-matplotlib`, `python3-tk`

## 설치

```bash
sudo apt install ros-jazzy-dynamixel-sdk python3-matplotlib python3-tk
sudo usermod -aG dialout $USER        # 시리얼 권한 (재로그인 필요)

mkdir -p ~/ax12_ws/src && cd ~/ax12_ws/src
git clone https://github.com/A-little-sunshine0721/ax12_control.git
cd ~/ax12_ws
colcon build --packages-select ax12_control
source install/setup.bash
```

> 설정 파일 기본 경로가 `~/ax12_ws/src/ax12_control/scripts/joints.json`입니다.
> 다른 위치에 설치했다면 실행 시 `-p joints_file:=<경로>`로 지정하세요.

## 초기 설정

```bash
cd ~/ax12_ws/src/ax12_control/scripts
python3 scan_and_test.py --scan          # 연결된 모터 ID 확인
python3 set_id.py <현재ID> <새ID>         # (필요 시) AX-12A ID 변경
python3 joint_tool.py limits --ids 1 2 3 4 5   # joints.json 범위를 AX-12A EEPROM 한계로 저장
```

### joints.json

관절 각도는 `모터 각도 = zero_deg + dir × 관절 각도`로 변환됩니다.

```json
{
  "2": {"zero_deg": 150.0, "dir":  1, "min_deg": -93.0, "max_deg": 93.0},
  "3": {"zero_deg": 150.0, "dir": -1, "min_deg": -93.0, "max_deg": 93.0},
  "6": {"zero_deg": 180.0, "dir":  1, "min_deg": -93.0, "max_deg": 93.0}
}
```

- 파일에 없는 모터는 기본값을 씁니다 (AX: 150°=0°, ±140° / XL: 180°=0°, ±170°).
- 명령 가능 범위는 `min/max_deg`보다 `cmd_margin`(3°) 안쪽입니다. 남는 3°는 오차 보정용 여유입니다.

### XL330 그리퍼 설정 (EEPROM, 1회)

XL330의 Current Limit(주소 38)이 낮으면 그리퍼가 힘을 못 냅니다. 1000~1750 mA로 설정하세요.
권장 전원 전압은 **5.0 V**입니다.

## 실행

```bash
ros2 run ax12_control arm_controller     # 터미널 1: 제어기
ros2 run ax12_control key_teleop         # 터미널 2: 키보드 조작
ros2 run ax12_control monitor_plot       # 터미널 3: 실시간 그래프
```

### 토픽으로 명령

```bash
ros2 topic pub --once /ax12/joint23/cmd std_msgs/msg/Float64 "{data: 30.0}"     # ID2 +30°, ID3 -30°
ros2 topic pub --once /ax12/joint1/cmd  std_msgs/msg/Float64 "{data: 20.0}"
ros2 topic pub --once /ax12/cmd_deg sensor_msgs/msg/JointState \
  "{name: ['joint1', 'joint23'], position: [10.0, -45.0]}"                      # 여러 관절 동시 (deg)
ros2 topic pub --once /ax12/joint6/current std_msgs/msg/Float64 "{data: 500.0}" # 그리퍼 힘 [mA]
ros2 topic pub --once /ax12/enable std_msgs/msg/Bool "{data: false}"            # 전체 토크 OFF
```

### 키보드 조작 (`key_teleop`)

| 키 | 동작 |
|---|---|
| `1`~`9` | 모터 ID 선택 (2·3은 joint23으로 함께) |
| `←` / `→` | 선택 관절 −/+ 이동 |
| `↑` / `↓` | 스텝 크기 (0.5 / 1 / 2 / 5 / 10 / 20°) |
| `0` / `z` | 선택 관절 0° / 전체 0° |
| `e` | 선택 관절 토크 ON/OFF |
| `s` / `h` / `g` / `l` | 원위치 저장 / 전체 복귀 / 선택 관절 복귀 / 보기 |
| `q` | 종료 |

원위치는 `scripts/home.json`에 저장되어 재시작 후에도 유지됩니다.

## 토픽

| 토픽 | 타입 | 내용 |
|---|---|---|
| `/ax12/<joint>/cmd` | Float64 | 관절 목표 각도 [deg] (입력) |
| `/ax12/cmd_deg` | JointState | 여러 관절 동시 명령, position 단위 deg (입력) |
| `/ax12/<joint>/enable`, `/ax12/enable` | Bool | 관절별 / 전체 토크 ON·OFF (입력) |
| `/ax12/joint6/current` | Float64 | 그리퍼 전류 제한 [mA] (입력) |
| `/joint_states` | JointState | 모든 모터 위치·속도·부하 (rad) |
| `/ax12/<joint>/debug` | Float64MultiArray | `[target, ref, q…, e…, corr…, (2축이면 sync)]` |
| `/ax12/<joint>/status` | Float64MultiArray | `[온도…, 전압…, 부하%(AX) / 전류 mA(XL)…]` |

## 제어 구조

```
ref(사다리꼴 궤적) → e_i = ref − q_i
I_i += ki · [e_i + kci · (e_i − e_j)] · dt      (정지 중에만, 불감대·anti-windup 적용)
u_i  = ref + I_i + kc · (e_i − e_j)
→ 모터 각도 = zero + dir · u_i → SyncWrite (AX: 주소 30 / XL: 주소 116)
```

- 하위 제어: AX-12A 내부 Compliance(P), XL330 내부 PID
- 상위 제어(PC): 공통 궤적 + 조건부 적분 + 교차 결합 → **Cross-Coupled PI 동기 제어**
- 정지 정밀도는 분해능이 한계입니다 (AX-12A 0.29°, XL330 0.088°).

## 주요 파라미터 (`arm_controller`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `ids` | [1,2,3,4,5,6] | 제어할 모터 ID |
| `groups` | ['2,3'] | 동기로 묶을 ID |
| `max_vel` / `max_acc` | 90 / 300 | 궤적 최대 속도 [deg/s] / 가속도 [deg/s²] |
| `ki` / `kci` / `kc` / `kp` | 2.0 / 1.0 / 0.1 / 0.0 | 적분 / 적분 동기 / 비례 동기 / 비례 (`kp + 2·kc < 0.5` 유지) |
| `deadband` / `deadband_xl` | 0.15 / 0.05 | 적분 불감대 [deg] |
| `cmd_margin` | 3.0 | 하드웨어 한계 대비 명령 범위 여유 [deg] |
| `gripper_ids` / `gripper_current` | [6] / 300 | 그리퍼 ID / 잡는 힘 [mA] |
| `xl_pos_p` / `xl_pos_i` / `xl_pos_d` | 800 / 0 / 0 | XL330 내부 위치 PID |
| `temp_warn` / `temp_stop` | 60 / 68 | 온도 경고 / 정지 [°C] |

```bash
ros2 run ax12_control arm_controller --ros-args -p max_vel:=60.0 -p gripper_current:=600.0
```

## scripts/ (ROS 없이 쓰는 도구)

| 파일 | 용도 |
|---|---|
| `scan_and_test.py` | ID 스캔(`--scan`), 30° 간격 동작 테스트 |
| `set_id.py` | AX-12A ID 변경 |
| `joint_tool.py` | 손으로 돌리며 각도 확인(`read`), EEPROM 범위 저장(`limits`), 설정 확인(`show`) |
| `diag.py` | 동작 불가 원인 진단 (AX-12A) |
| `comp_tool.py` | 공용 통신 함수 (위 도구들이 사용) |

> 스크립트는 시리얼 포트를 직접 사용하므로 **`arm_controller`를 끈 상태**에서 실행하세요.

## 문제 해결

| 증상 | 확인 |
|---|---|
| `Package 'ax12_control' not found` | `source ~/ax12_ws/install/setup.bash` |
| `Permission denied: /dev/ttyACM0` | dialout 그룹 추가 후 재로그인 |
| 특정 ID가 인식되지 않음 | 전원, 케이블, 보레이트(1 Mbps), ID 중복 확인 |
| 관절이 `비활성` | 로그의 원인(과부하·과열 등) 확인 후 `/ax12/<joint>/enable` 에 true |
| 범위 밖 명령 경고 | `joints.json` 범위 확인. 명령은 무시되고 현재 위치 유지 |
| 정지 시 "삐" 소리 | `-p deadband:=0.3 -p margin:=2 -p slope:=64` |
| 그리퍼가 안 움직임 | XL330 Current Limit(38), PID(84/80), 전원 5V 확인 |
| 2·3번이 진동 | `kc`를 낮춤 (`kp + 2·kc < 0.5`) |

## License

MIT
