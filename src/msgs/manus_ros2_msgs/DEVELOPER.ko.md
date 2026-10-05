# `manus_ros2_msgs` 개발자 가이드

> Manus glove 데이터의 ROS2 메시지 정의. `manus_data_publisher`(publisher)와 `manus_tesollo`(subscriber) 사이의 계약.

---

## 1. 패키지 개요

4개 메시지:
- `ManusGlove` — glove 전체 데이터 (skeleton + ergonomics + raw sensor)
- `ManusErgonomics` — 단일 ergonomic 측정값 (type + value)
- `ManusRawNode` — skeleton의 단일 노드 (위치 + 회전 + 메타)
- `ManusVibrationCommand` — 손가락별 진동 명령 (ManusSDK 3.2.1부터, publisher가 구독)

---

## 2. 디렉토리 구조

```
src/msgs/manus_ros2_msgs/
├── CMakeLists.txt
├── package.xml
└── msg/
    ├── ManusErgonomics.msg
    ├── ManusGlove.msg
    ├── ManusRawNode.msg
    └── ManusVibrationCommand.msg
```

---

## 3. 빌드

```bash
cd manusdelto
colcon build --packages-select manus_ros2_msgs
source install/setup.bash
```

---

## 4. 메시지별 상세

### 4.1 `ManusErgonomics.msg`

```
string type           # ergonomic 항목 이름 (예: "IndexMCPStretch")
float32 value         # 측정값 (degrees, Manus SDK 단위)
```

손당 20개 항목. 검지~새끼는 `{Finger}Spread`/`MCPStretch`/`PIPStretch`/`DIPStretch`, 엄지는 해부학 명칭대로 `ThumbSpread`/`ThumbCMCStretch`/`ThumbMCPStretch`/`ThumbIPStretch` (ManusSDK 3.2.1부터. 이전 SDK의 `ThumbMCPSpread`/`ThumbMCPStretch`/`ThumbPIPStretch`/`ThumbDIPStretch`와 슬롯은 같고 이름만 바뀌었다 — `ThumbMCPStretch`는 이름이 같지만 다른 슬롯이다). 실제로 읽는 키는 `manus_tesollo_node.py`의 `_cb`(q_deg 목록) 참조.

### 4.2 `ManusRawNode.msg`

```
int32 node_id                       # SDK 내부 노드 ID
int32 parent_node_id                # 부모 노드 ID (skeleton tree)
string joint_type                   # 예: "Carpus", "ThumbMCP", "IndexPIP"
string chain_type                   # 예: "Thumb", "Index", ...
geometry_msgs/Pose pose             # 노드의 SE3 (Manus 좌표계)
```

skeleton에서 한 노드. 보통 손가락 마디 또는 손목 base.

### 4.3 `ManusGlove.msg`

```
int32 glove_id                                  # Manus dongle ID (uint32 cast)
string side                                     # "Left" 또는 "Right"
int32 raw_node_count                            # raw_nodes 길이
ManusRawNode[] raw_nodes                        # skeleton (가변 길이)
int32 ergonomics_count                          # ergonomics 길이
ManusErgonomics[] ergonomics                    # 각도 측정 (가변 길이)
geometry_msgs/Quaternion raw_sensor_orientation # raw IMU 방향
int32 raw_sensor_count                          # raw_sensor 길이
geometry_msgs/Pose[] raw_sensor                 # raw sensor pose (가변 길이)
```

count 필드는 array 길이와 일치해야 함. (대부분 ROS2 인터페이스는 array가 자체 길이를 알지만 본 메시지는 명시적 count 보관.)

### 4.4 `ManusVibrationCommand.msg`

```
float32[5] intensities   # Thumb, Index, Middle, Ring, Pinky
```

publisher가 `/manus_glove_<idx>/vibration_cmd`로 구독한다. 0~1로 클램프되고, 햅틱 글러브가 아니면 무시된다.

---

## 5. 소비자

| 패키지 | 토픽 | 처리 |
|---|---|---|
| `manus_tesollo` | `/manus_glove_0`, `/manus_glove_1` | `side`로 좌/우 식별. ergo는 `ergonomics`, dex/dex_vector는 `raw_nodes`(skeleton) 사용 → DG5F 관절각 |
| (옵션) `client_scripts/manus_data_viz.py` | 동일 | matplotlib 시각화 (디버그) |

---

## 6. 흔한 함정

- **`raw_node_count` ≠ `raw_nodes.size()`** : SDK 콜백 시점에 데이터가 일부만 들어왔을 때 발생 가능. consumer는 `.size()` 우선 사용 권장.
- **`ergonomics`의 type 키**: SDK 버전에 따라 키 이름이 다를 수 있음. 3.2.1에서 엄지 키가 바뀌었다(§4.1). `manus_tesollo_node.py`가 읽는 키와 맞는지 확인 — 없는 키는 `ergo.get(..., 0.0)`이 조용히 0을 돌려준다.
- **side가 빈 문자열**: glove discovery 미완료 상태에서 publish하면 발생. `manus_tesollo`는 left/right가 아닌 side 메시지를 경고 후 무시.

---

## 7. 확장 / 수정 가이드

### 7.1 새 ergonomic 항목 추가
1. `ManusErgonomics`는 type+value 쌍이라 메시지 구조 변경 없음
2. `manus_tesollo_node.py`의 `_cb`에 새 키 추가
3. SDK가 새 키를 publish하는지 확인

### 7.2 raw skeleton 사용
- `manus_tesollo`의 dex/dex_vector 리타게터가 `raw_nodes`로 MANO 21-keypoint를 채운다(`retargeters/dex.py`)

### 7.3 시간 stamp 추가
- 현재 메시지에 `Header`가 없음. 정확한 timing이 필요하면 `std_msgs/Header header` 추가 + manus_data_publisher 측에서 채우기.

---

## 8. 연관 패키지

- `manus_ros2` (input) — 유일한 publisher
- `manus_tesollo` (core) — subscriber
