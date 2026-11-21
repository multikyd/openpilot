# KisaPilot 설정 안내

현재 KYD 코드의 설정 화면 스키마에 있는 전체 91개 항목을 설명합니다. 화면/API의 한글·영문 설명 원본은 [param_schema.json](param_schema.json)입니다. 아래 기본값은 코드의 신규 설치 기본값이며 실제 장치에 저장된 값과 다를 수 있습니다.

파라미터 정리 커밋 `52a402c`는 설명·표시와 미사용 항목 제거이며 기존 유효 설정의 기본값·허용 범위·주행 계산은 바꾸지 않습니다. 설정값이 0이라고 항상 기능 전체가 꺼지는 것은 아닙니다. 항목별 의미를 확인하세요.

## 제거한 미사용 항목

| 키 | 확인 결과 |
|---|---|
| `SoundVolumeAdjustEngage` | 별도 인게이지 음량을 읽는 코드 없음. 실제 음량은 `AlertVolumePct` 사용. |
| `RecordRoadCam` | 녹화 방식을 선택하는 소비 코드 없음. |
| `KisaBlindSpotDetect` | 화면 표시 소비 코드 없음. 실제 BSD 수신·판정과는 별도인 과거 UI 설정. |
| `RecordingTimePerVideo` / `RecordingMaxFiles` | 파일 분할·보관에 사용하는 코드 없음. |
| `ShowDebugUI` / `ShowRadarInfo` / `ShowPlotMode` | 현재 UI에 소비 코드 없음. |
| `PathOffset` | 읽어서 저장만 하고 경로 계산에 쓰지 않던 두 줄과 함께 제거. |
| `LatMpcInputOffset` | 등록과 미사용 테스트 더미값만 남아 있어 제거. |
| `KisaSpeedTable` | 키 소비 코드 없음. 동명의 실제 `KisaSpeedTable` 클래스는 유지. |

등록 목록은 286개에서 275개, 화면 스키마는 100개에서 91개로 정리했습니다. 내부 상태·캐시·외부 앱 통신 키는 화면에 없다는 이유만으로 제거하지 않았습니다. `PutPrebuiltOn`, `CarList`, `GitCommits`처럼 부팅 스크립트나 셸에서 쓰는 키도 유지합니다. 차량에 저장된 과거 파라미터 파일을 일괄 삭제하는 작업은 포함하지 않습니다.

## 위치가 왼쪽으로 갔다 복귀할 수 있는 코드 경로

- `LaneCenterOffset`이 음수일 때 모델모드의 `LanePositionGuard`는 특정 속도·차선 조건에서 왼쪽 복귀 보정을 추가합니다. 기하 정보·신뢰도·운전자 개입 등에 따라 보정이 해제됩니다. [model_lane_guard.py](../controls/lib/model_lane_guard.py)
- 레인모드에서는 자동 차로 위치 프로필과 수동 중심 선호가 적용됩니다. 자동 프로필은 차로 분류를 약 1.5초 확인하고 목표 보정을 서서히 이동시킵니다. 모델모드로 바뀌면 이 항은 서서히 해제됩니다. [lane_path.py](../controls/lib/lane_path.py)
- 근거리 앞차 및 예측 커브 속도가 레인/모델 선택에 관여합니다. 모드마다 다른 경로·보정이 이어지면 움직였다 복귀하는 느낌이 생길 수 있습니다. [close_lead_mode.py](../controls/lib/close_lead_mode.py), [controlsd.py](../controls/controlsd.py)

이는 현재 코드로 확인한 원인 후보입니다. 실제 원인 확정에는 해당 시점의 lateralControlMode, 모델/차선 경로, 차선 확률, 조향 목표와 장치에 저장된 보정값을 함께 확인해야 합니다. 파라미터 정리 커밋 `52a402c`에서는 조향 제어를 변경하지 않았습니다. 후속 수정에서는 레인모드의 중복 `LanePositionGuard` 연결을 제거했습니다. 레인모드 진입 시 이 보정 상태를 초기화하고 레인플래너의 위치·경계 보정을 사용합니다. 모델모드의 기존 보정은 유지합니다. 기존 controlsState.debugMsg1의 `LPG=`는 위치 복귀 보정, `LCF=model`은 근거리 앞차에 따른 모델모드 유지, `MLG=`는 모델 경계 보정을 확인할 단서입니다.

## UI/Start

### HYUNDAI: 카메라 SCC · `HyundaiCameraCruiseMode`

범위: 0 ~ 3 (단계 1) / 코드 기본값: `1`

현대차 SCC 배선과 제어 방식을 선택합니다. 0: 카메라 SCC 강제 설정 없음, 1: 카메라 SCC, 2: 카메라 SCC와 크루즈 상태 동기화, 3: 순정 종방향 제어용 버튼 전달. 실제 배선·지원 차종에 맞춰 설정하고 재시작해야 합니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py), [hyundaicanfd.py](../../../opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py), [interface.py](../../../opendbc_repo/opendbc/car/hyundai/interface.py), [ui_state.py](../../../openpilot/selfdrive/ui/ui_state.py)

### CANFD: HDA2 모드 · `CanfdHighwayAssistMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `2`

CAN-FD 차량의 HDA2 조향 메시지 구성을 선택합니다. 0은 강제 설정 없음이며 1과 2는 현재 코드에서 모두 HDA2 구성을 선택합니다. 2를 선택해도 사각지대 센서가 새로 활성화되지는 않으며 실제 CAN 수신 여부로 감지합니다. 변경 후 재시작해야 합니다.

사용처: [interface.py](../../../opendbc_repo/opendbc/car/hyundai/interface.py)

### 레이더 트랙 사용 · `EnableRadarTracks`

범위: -1 ~ 3 (단계 1) / 코드 기본값: `3`

레이더 입력 방식을 선택합니다. -1과 0은 SCC 기반 입력, 1~3은 지원 차량의 레이더 트랙 입력입니다. 현재 레이더 파서는 1~3을 같은 트랙 모드로 처리하며, 2·3의 별도 SCC/끼어들기 기능은 이 값만으로 추가되지 않습니다. 배선·차종 지원이 필요하며 변경 후 재시작해야 합니다.

사용처: [interface.py](../../../opendbc_repo/opendbc/car/hyundai/interface.py), [radar_interface.py](../../../opendbc_repo/opendbc/car/hyundai/radar_interface.py), [radard.py](../../../openpilot/selfdrive/controls/radard.py)

### 자동 크루즈 제어 · `AutoCruiseResumeMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `0`

페달을 놓거나 앞차가 가까워지는 조건에서 크루즈 자동 재개와 소프트홀드를 허용합니다. 0은 끄기, 1·2는 자동 제어 허용입니다. 실제 재개는 기어·취소 대기·차량 상태 조건을 함께 확인하며, 순정 SCC 차량에서는 소프트홀드가 적용되지 않습니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py), [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 크루즈 자동 ON 거리(m) · `AutoResumeLeadDistanceM`

범위: 0 ~ 25 (단계 0.5) / 코드 기본값: `4.0`

페달을 놓은 상태에서 크루즈 자동 재개를 판단하는 앞차 거리입니다(단위 m). 0은 거리 조건으로 재개하지 않습니다. AutoCruiseResumeMode 및 차량 상태 조건을 만족해야 하며, 정차 목표 간격을 설정하는 항목은 아닙니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 자동 인게이지 · `StartupEngagementMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `0`

초기 자동 활성화 범위를 선택합니다. 0: 자동 활성화 없음, 1: 조향, 2: 조향과 크루즈. 차량 준비 상태와 기존 활성화 제한을 통과해야 작동하며, 변경 후 차량 인터페이스 재시작이 필요합니다.

사용처: [carstate.py](../../../opendbc_repo/opendbc/car/hyundai/carstate.py), [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 자동 GasTok 활성화 속도 · `PedalTapResumeThreshold`

범위: 0 ~ 200 (단계 5) / 코드 기본값: `0`

가속 페달을 짧게 밟았다 놓는 GasTok 및 페달 해제 자동 재개 조건에 사용하는 속도 기준입니다(단위 km/h). 값이 높을수록 저속에서 해당 재개 조건이 제한됩니다. AutoCruiseControl이 꺼져 있으면 자동 재개하지 않습니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 사운드 볼륨(기본 100%) · `AlertVolumePct`

범위: 5 ~ 200 (단계 5) / 코드 기본값: `100`

안내음·경고음 전체에 곱하는 음량 비율입니다. 100은 기본 음량, 50은 절반입니다. soundd 시작 시 읽으므로 변경 반영에는 사운드 서비스 재시작이 필요합니다.

사용처: [soundd.py](../../../openpilot/selfdrive/ui/soundd.py)

### 화면 밝기(%) · `DisplayBrightnessPct`

범위: 10 ~ 100 (단계 10) / 코드 기본값: `50`

화면의 사용자 밝기 비율입니다(단위 %). 화면 밝기 제어에서 사용하는 값이며 카메라 노출에는 영향을 주지 않습니다. UI 시작 시 읽으므로 변경 후 UI를 재시작해야 합니다.

사용처: [ui_state.py](../../../openpilot/selfdrive/ui/ui_state.py)

### HDP(CCNC) 사용 · `StockHudAssistDisplayMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `0`

지원되는 순정 HUD에 보내는 HDP 상태 표시를 선택합니다. 0: 추가 표시 안 함, 1: 크루즈와 내비 안내가 함께 활성화될 때, 2: 크루즈 활성화 시. 실제 순정 HDP 자율주행 기능을 켜는 설정은 아닙니다.

사용처: [hyundaicanfd.py](../../../opendbc_repo/opendbc/car/hyundai/hyundaicanfd.py)

### 크루즈 속도 자동 동기화 · `SyncSetSpeedOnAccelerator`

범위: 켜기/끄기 / 코드 기본값: `1`

가속 페달을 일정 시간 이상 밟는 동안 현재 속도가 설정 속도를 넘으면 크루즈 설정 속도를 현재 속도에 맞춥니다. 주변 차량 속도를 따라 설정 속도를 계속 올리는 기능은 아닙니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py), [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

## LAT

### 카메라 오프셋 · `CameraLateralOffsetM`

범위: -5 ~ 5 (단계 0.01) / 코드 기본값: `0.04`

차선 경로 계산에 쓰는 카메라 기준 오프셋입니다(단위 m). 수동 항은 음수가 왼쪽, 양수가 오른쪽 방향이며, 기존 계산은 0.04m를 보정 기준으로 사용합니다. 자동 차로 위치 프로필이 이 값을 보상할 수 있으므로 표시값과 실제 횡이동량은 같지 않습니다. 모델 직접 조향의 고정 이동량도 아닙니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py), [lane_path.py](../../../openpilot/selfdrive/controls/lib/lane_path.py), [lateral_planner.py](../../../openpilot/selfdrive/controls/lib/lateral_planner.py), [ldw.py](../../../openpilot/selfdrive/controls/lib/ldw.py)

### 차선 중심 위치 선호(m) · `LaneCenterOffset`

범위: -0.1 ~ 0.1 (단계 0.01) / 코드 기본값: `-0.03`

차선 중심에 더하는 수동 위치 선호입니다(단위 m). 음수는 왼쪽, 양수는 오른쪽이며 레인모드에서 커브·차선변경 시 줄어듭니다. 레인모드에서는 레인플래너만 위치를 보정하며 추가 LanePositionGuard는 사용하지 않습니다. 모델모드에서는 음수 설정에 한해 일정 속도·차선 조건에서 왼쪽 복귀 보정에도 사용합니다. 0은 이 수동 선호만 해제하며 자동 위치·경계 보정은 남습니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py), [lane_path.py](../../../openpilot/selfdrive/controls/lib/lane_path.py)

### Steer Ratio(Manual) · `ManualSteeringRatio`

범위: 0 ~ 30 (단계 0.05) / 코드 기본값: `0.0`

수동 조향비입니다. 1보다 큰 값은 실시간 추정 조향비 대신 사용하며, 0~1은 실시간 추정값에 SteerRatioRate를 곱해 사용합니다. 조향 계산의 차량 모델에 반영되며 핸들 각도 자체를 고정하지 않습니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py)

### Live Steer Ratio 적용 비율(%) · `LiveSteeringRatioPct`

범위: 30 ~ 170 (단계 1) / 코드 기본값: `100`

실시간 추정 조향비에 곱하는 비율입니다(단위 %). 100은 추정값 그대로 사용합니다. CustomSR가 1보다 크면 수동 조향비가 우선하므로 이 비율은 적용되지 않습니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py)

### Camera Yaw Trim Deg · `CameraYawTrimDeg`

범위: -2 ~ 2 (단계 0.01) / 코드 기본값: `0.0`

모델 입력 카메라의 좌우 회전 보정입니다(단위 도). 0은 추가 YAW 보정 없음입니다. 모델 영상 변환과 캘리브레이션 처리에 관여하므로 차량을 일정 거리만큼 옮기는 오프셋과 다릅니다. 0이 아니면 기존 캘리브레이션 갱신을 제한하는 처리도 적용됩니다.

사용처: [calibrationd.py](../../../openpilot/selfdrive/locationd/calibrationd.py), [modeld.py](../../../openpilot/selfdrive/modeld/modeld.py)

### Steer Actuator Delay · `SteeringDelaySec`

범위: 0 ~ 1 (단계 0.05) / 코드 기본값: `0.0`

조향 명령과 차량 응답 사이의 예측 지연 시간입니다(단위 초). 0은 지연 없음이 아니라 실시간 lateralDelay 추정값 사용입니다. 양수는 수동 지연값으로 사용하며 조향 예측과 모델 입력에 반영됩니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py), [modeld.py](../../../openpilot/selfdrive/modeld/modeld.py)

### Lateral 스무딩 시간 · `LateralSmoothingSec`

범위: -0.01 ~ 3 (단계 0.01) / 코드 기본값: `0.13`

조향 목표 스무딩의 기본 시간입니다(단위 초). 레인 제어에서는 음수일 때 코드 기본값을 사용합니다. 모델 처리에서는 0 이하이면 추가 스무딩을 끄고, 양수이면 궤적 불확실성에 따라 늘리되 최대 0.60초로 제한합니다. 크게 하면 변화는 완만해지지만 반응도 늦어질 수 있습니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py), [modeld.py](../../../openpilot/selfdrive/modeld/modeld.py)

### Steer Max · `SteeringTorqueLimit`

범위: 0 ~ 1024 (단계 1) / 코드 기본값: `0`

지원되는 현대차 토크 조향 송신에서 최대 조향 토크을 재정의합니다(CAN 원시 단위). 0은 차종 기본값입니다. 각도 조향 방식에는 동일하게 적용되지 않을 수 있으며 Panda 송신 제한은 별도로 유지됩니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py)

### Steer Delta Up · `SteeringTorqueRiseLimit`

범위: 0 ~ 50 (단계 1) / 코드 기본값: `0`

지원되는 현대차 토크 조향 송신에서 조향 토크 증가 폭을 재정의합니다(CAN 원시 단위). 0은 차종 기본값입니다. 각도 조향 방식에는 동일하게 적용되지 않을 수 있으며 Panda 송신 제한은 별도로 유지됩니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py)

### Steer Delta Down · `SteeringTorqueFallLimit`

범위: 0 ~ 50 (단계 1) / 코드 기본값: `0`

지원되는 현대차 토크 조향 송신에서 조향 토크 감소 폭을 재정의합니다(CAN 원시 단위). 0은 차종 기본값입니다. 각도 조향 방식에는 동일하게 적용되지 않을 수 있으며 Panda 송신 제한은 별도로 유지됩니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py)

### 레이더 컷인 감도 계수 · `RadarCutInLateralFactor`

범위: 0 ~ 10 (단계 0.1) / 코드 기본값: `2.0`

레이더 차량의 횡방향 움직임으로 끼어들기를 예측하는 감도입니다. 예측 시간과 판정 문턱을 함께 조절하며, 0은 이 예측을 끕니다. 레이더 자체나 기존 앞차 추적을 끄는 설정은 아닙니다.

사용처: [longitudinal_planner.py](../../../openpilot/selfdrive/controls/lib/longitudinal_planner.py), [radard.py](../../../openpilot/selfdrive/controls/radard.py)

### 코너 레이더 설정 · `EnableCornerRadar`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `1`

지원 코너레이더 CAN 정보를 사용합니다. 0: 사용 안 함, 1: 객체 정보 수신, 2: 수신한 객체를 끼어들기 판단에도 사용. 끼어들기 예측에는 RadarLatFactor도 0보다 커야 합니다. 장착·배선·메시지 지원이 필요하며 수신 구성 변경 후 재시작해야 합니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py), [radar_interface.py](../../../opendbc_repo/opendbc/car/hyundai/radar_interface.py), [longitudinal_planner.py](../../../openpilot/selfdrive/controls/lib/longitudinal_planner.py), [radard.py](../../../openpilot/selfdrive/controls/radard.py)

### 왼쪽 엣지 치우침 조정 · `LeftEdgeOffset`

범위: -1 ~ 1 (단계 0.01) / 코드 기본값: `-0.05`

왼쪽 도로경계가 인식될 때 사용하는 레거시 모델모드 경로 계산 보정입니다(단위 m, 음수 왼쪽·양수 오른쪽). 0은 해당 보정을 끕니다. 현재 모델 직접 조향은 이 경로를 그대로 사용하지 않으므로 모델모드 차량 위치를 고정 이동시키는 효과는 보장되지 않습니다.

사용처: [lane_path.py](../../../openpilot/selfdrive/controls/lib/lane_path.py)

### 오른쪽 엣지 치우침 조정 · `RightEdgeOffset`

범위: -1 ~ 1 (단계 0.01) / 코드 기본값: `0.05`

오른쪽 도로경계가 인식될 때 사용하는 레거시 모델모드 경로 계산 보정입니다(단위 m, 음수 왼쪽·양수 오른쪽). 0은 해당 보정을 끕니다. 현재 모델 직접 조향은 이 경로를 그대로 사용하지 않으므로 모델모드 차량 위치를 고정 이동시키는 효과는 보장되지 않습니다.

사용처: [lane_path.py](../../../openpilot/selfdrive/controls/lib/lane_path.py)

## LONG

### Longitudinal P Gain · `LongitudinalSpeedKp`

범위: 0 ~ 1.5 (단계 0.05) / 코드 기본값: `0.5`

종방향 속도 오차에 반응하는 P 게인입니다. 값이 클수록 같은 오차에 대한 즉각 반응이 커집니다. 차량 기본 P·I 게인 테이블이 모두 단일 기준점인 경우에만 수동값을 적용하며, 속도별 테이블이 있는 차종은 기본 테이블을 사용합니다.

사용처: [longcontrol.py](../../../openpilot/selfdrive/controls/lib/longcontrol.py)

### Longitudinal I Gain · `LongitudinalSpeedKi`

범위: 0 ~ 2 (단계 0.01) / 코드 기본값: `0.1`

누적 속도 오차를 보정하는 종방향 I 게인입니다. 0은 적분항만 끄며 종방향 제어 전체를 끄지 않습니다. 차량 기본 P·I 게인 테이블이 모두 단일 기준점일 때만 수동값이 적용됩니다.

사용처: [longcontrol.py](../../../openpilot/selfdrive/controls/lib/longcontrol.py)

### Longitudinal Actuator Delay · `LongitudinalDelaySec`

범위: 0 ~ 2 (단계 0.05) / 코드 기본값: `0.2`

가감속 명령부터 차량 반응까지의 예측 지연입니다(단위 초). 모델과 종방향 계획이 이 시간만큼 앞선 상태를 고려합니다. 정차 거리나 감속 강도를 직접 지정하는 값은 아닙니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py), [longitudinal_planner.py](../../../openpilot/selfdrive/controls/lib/longitudinal_planner.py), [modeld.py](../../../openpilot/selfdrive/modeld/modeld.py)

### 모델 정지 판정 속도(m/s) · `ModelStopThresholdMps`

범위: 0.01 ~ 1 (단계 0.05) / 코드 기본값: `0.05`

모델 궤적에서 정지 여부를 판단하는 저속 기준입니다(단위 m/s, 0.25m/s는 약 0.9km/h). 기준을 높이면 정지로 분류하는 속도 범위가 넓어집니다. 감속 강도나 목표 정차 간격을 설정하는 항목은 아닙니다.

사용처: [longitudinal_planner.py](../../../openpilot/selfdrive/controls/lib/longitudinal_planner.py), [modeld.py](../../../openpilot/selfdrive/modeld/modeld.py)

### 레이더 반응 계수 · `RadarReactionFactor`

범위: 0 ~ 2 (단계 0.1) / 코드 기본값: `1.1`

앞차 가속도 필터의 반응 문턱과 예측 감쇠에 쓰는 계수입니다. 작게 하면 작은 가속도 변화에도 예측 상태가 더 빨리 바뀔 수 있습니다. 0도 계산에 사용하는 값이며 레이더를 끄지 않습니다.

사용처: [radard.py](../../../openpilot/selfdrive/controls/radard.py)

### 정지 상태 감속 목표(m/s²) · `StoppingStateAccelMps2`

범위: -1 ~ 0 (단계 0.05) / 코드 기본값: `0.0`

종방향 제어가 정지 상태로 전환할 때 사용하는 감속 목표를 재정의합니다(단위 m/s²). 음수가 클수록 강한 제동이며 0은 차량 기본 stopAccel 사용입니다. 실제 명령에는 정지 상태 전환·감속 변화 제한도 적용됩니다.

사용처: [longcontrol.py](../../../openpilot/selfdrive/controls/lib/longcontrol.py)

### 정지 거리(m) · `KisaStandstillDistance`

범위: 0.3 ~ 10 (단계 0.1) / 코드 기본값: `3.3`

앞차 추종 계산의 기본 정차 여유 거리입니다(단위 m). 속도에 따른 차간 시간·제동 거리와 함께 사용합니다. 실제 최종 간격은 앞차 추정과 정지 제어에 따라 달라지므로 이 거리에서 즉시 멈추는 스위치가 아닙니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 선행 차량 저크 계수 · `LeadJerkWeight`

범위: 0 ~ 1 (단계 0.05) / 코드 기본값: `0.65`

앞차의 가속도 변화율(저크)을 미래 움직임 예측에 반영하는 비율입니다. 0은 해당 저크 보정을 사용하지 않습니다. 앞차 추적·거리 기반 감속은 계속 작동합니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 앞차 저크별 가속 변화 비용 · `LeadJerkAccelChangeCost`

범위: 10 ~ 500 (단계 5) / 코드 기본값: `125`

앞차 저크가 커질 때 MPC에 사용하는 가속도 변화 비용입니다. 기본 비용과 이 값 사이를 상황에 따라 보간합니다. 비용이 크면 가속도 변화를 더 억제하지만, 정차 마지막 충격이나 목표 거리를 직접 지정하지는 않습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 앞차 위험 거리 제약 계수 · `LeadDangerFactor`

범위: 0.5 ~ 1.5 (단계 0.01) / 코드 기본값: `0.75`

MPC의 앞차 위험 거리 제약에 곱하는 계수입니다. 크게 하면 거리 제약이 더 보수적으로 작용합니다. 감속 시작 시점만을 독립적으로 정하는 값이 아니며 실제 감속은 속도·상대속도·다른 제약과 함께 결정됩니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 0km/h · `MaxAccelAt0Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `1.65`

0km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 10km/h · `MaxAccelAt10Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `1.55`

10km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 40km/h · `MaxAccelAt40Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `0.8`

40km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 60km/h · `MaxAccelAt60Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `0.6`

60km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 80km/h · `MaxAccelAt80Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `0.4`

80km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 110km/h · `MaxAccelAt110Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `0.3`

110km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 가속 상한 기준점 140km/h · `MaxAccelAt140Kph`

범위: 0 ~ 2.5 (단계 0.05) / 코드 기본값: `0.2`

140km/h 기준점의 최대 가속도입니다(단위 m/s²). 기준 속도는 0·10·40·60·80·110·140km/h이며 사이 속도에서는 이웃 값 사이를 선형 보간합니다. 140km/h 이상은 마지막 값을 사용합니다. 실제 가속 명령은 앞차·주행 모드·커브 제한 등에 의해 더 낮아질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

## Button/Gap/Stop

### 크루즈 버튼 동작 모드 · `CruiseButtonMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `0`

크루즈 +/- 버튼의 짧게·길게 누름 동작을 선택합니다. 0은 기본 증감 단위, 1·2는 확장 감속 단위도 사용합니다. 2에서는 설정 속도보다 현재 속도가 높을 때 감속 버튼으로 현재 속도를 반영하는 분기가 추가됩니다. 활성 상태·누름 길이에 따라 재개·감속 동작이 달라집니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 취소 버튼 동작 모드 · `CancelButtonMode`

범위: 0 ~ 1 (단계 1) / 코드 기본값: `0`

취소 버튼의 해제 범위입니다. 0은 크루즈 취소, 1은 크루즈 취소와 함께 조향 활성 상태도 해제합니다. 운전자 취소 상태를 기록하여 자동 재개 판단에도 사용합니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### LFA 버튼 확장 모드 · `LfaButtonMode`

범위: 0 ~ 1 (단계 1) / 코드 기본값: `0`

LFA 버튼의 짧은 누름 기능입니다. 0은 조향 활성 상태 전환, 1은 추가 감속 상태 요청입니다. 현재 화면 범위는 0~1이며 감속 동작에는 기존 크루즈 상태 조건이 적용됩니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 크루즈 속도 단위(기본) · `CruiseButtonBasicStep`

범위: 1 ~ 20 (단계 1) / 코드 기본값: `10`

크루즈 속도 버튼의 기본 증감 폭입니다. 미터법에서는 km/h 단위이며, 비미터법에서는 버튼 계산에서 mph에 맞춰 환산합니다. 길게 누름·확장 동작은 CruiseSpeedUnit과 버튼 모드도 함께 사용합니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 크루즈 속도 단위(확장) · `CruiseButtonExtendedStep`

범위: 1 ~ 20 (단계 1) / 코드 기본값: `10`

길게 누르기 및 확장 감속 동작에서 사용하는 속도 단위입니다. 미터법에서는 km/h이며 해당 배수의 속도로 맞추는 분기에도 사용합니다. 모든 짧은 버튼 입력에 일괄 적용되지는 않습니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 에코 크루즈 보정 · `CruiseRecoverySpeedBoost`

범위: 0 ~ 10 (단계 1) / 코드 기본값: `0`

설정 속도보다 느린 상태에서 재가속할 때 계획 속도에 일시적으로 더하는 값입니다(단위 km/h). 원래 목표 속도에 도달하면 해제합니다. 0은 끄기이며 고정된 도로 제한 속도 오프셋과는 별도입니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 자동 속도 상승 비율 · `LeadSpeedIncreaseLimitPct`

범위: 0 ~ 200 (단계 10) / 코드 기본값: `10`

앞차를 따라 설정 속도를 자동으로 높이는 기능의 도로 제한 속도 배율입니다(단위 %). 0은 끄기, 100은 도로 제한 속도, 110은 제한 속도의 1.1배를 기준으로 사용합니다. 앞차·페달·일시정지 조건을 만족할 때만 작동하며 도로속도 오프셋과는 별도입니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 차간 거리 GAP 1 · `FollowingTimeGap1`

범위: 0.7 ~ 3 (단계 0.05) / 코드 기본값: `0.9`

차간 단계 1의 기본 추종 시간입니다(단위 초). 속도에 곱해 거리 항으로 사용하고 정차 여유·제동 거리 등을 더합니다. 속도별 GAP·동적 GAP 설정에 따라 실제 사용 시간은 달라질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 차간 거리 GAP 2 · `FollowingTimeGap2`

범위: 0.7 ~ 3 (단계 0.05) / 코드 기본값: `1.1`

차간 단계 2의 기본 추종 시간입니다(단위 초). 속도에 곱해 거리 항으로 사용하고 정차 여유·제동 거리 등을 더합니다. 속도별 GAP·동적 GAP 설정에 따라 실제 사용 시간은 달라질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 차간 거리 GAP 3 · `FollowingTimeGap3`

범위: 0.7 ~ 3 (단계 0.05) / 코드 기본값: `1.3`

차간 단계 3의 기본 추종 시간입니다(단위 초). 속도에 곱해 거리 항으로 사용하고 정차 여유·제동 거리 등을 더합니다. 속도별 GAP·동적 GAP 설정에 따라 실제 사용 시간은 달라질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 차간 거리 GAP 4 · `FollowingTimeGap4`

범위: 0.7 ~ 3 (단계 0.05) / 코드 기본값: `1.6`

차간 단계 4의 기본 추종 시간입니다(단위 초). 속도에 곱해 거리 항으로 사용하고 정차 여유·제동 거리 등을 더합니다. 속도별 GAP·동적 GAP 설정에 따라 실제 사용 시간은 달라질 수 있습니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 속도별 차간거리 설정 · `SpeedBasedFollowingGap`

범위: -3 ~ 50 (단계 1) / 코드 기본값: `0`

0은 선택한 GAP의 기본 시간 사용입니다. -1·-2·-3은 각각 [0,30,60,90]·[0,40,80,120]·[0,50,100,150]km/h에서 GAP 1~4 시간을 보간합니다. 양수는 저속 추종 시간을 해당 비율(%)만큼 줄이고 100km/h에 가까워질수록 원래 시간으로 복귀합니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 동적 차간 거리 제어 · `DynamicTFollow`

범위: 0 ~ 1 (단계 0.05) / 코드 기본값: `0.0`

앞차 저크에 따라 기본 추종 시간에 더하거나 빼는 동적 보정량입니다(단위 초). 앞차 감속 변화에는 여유를 늘리고 가속 변화에는 줄입니다. 0은 이 보정만 끕니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 차로 변경 시 동적 GAP 제어 · `DynamicTFollowLC`

범위: 0 ~ 1 (단계 0.05) / 코드 기본값: `1.0`

차선변경 desire가 활성화될 때 추종 시간과 저크 계수에 곱하는 비율입니다. 1은 원래 값, 작을수록 간격·변화 비용을 줄입니다. 계산에서 최소 0.2로 제한하므로 0은 끄기가 아니라 0.2배로 적용됩니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 주행 모드 선택 · `DrivingProfile`

범위: 1 ~ 4 (단계 1) / 코드 기본값: `3`

1 ECO, 2 SAFE, 3 NORMAL, 4 HIGH 주행 모드입니다. 가속 상한과 추종·신호 정지 판단에 영향을 줍니다. 현재 HIGH는 가속 비율을 높이며 모델 신호 정지 상태를 사용하지 않는 분기도 있으므로 단순 화면 테마 설정이 아닙니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py), [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 주행 모드 자동 전환 · `AdaptiveDrivingProfile`

범위: 0 ~ 1 (단계 1) / 코드 기본값: `0`

NORMAL 모드에서 상황에 따른 SAFE 성향 자동 조절을 허용합니다. 운전자가 주행 중 모드를 직접 바꾸면 해당 세션의 자동 모드 적용이 제한될 수 있습니다. 설정 속도를 임의로 높이는 기능은 아닙니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 신호등 인식 모드 · `TrafficSignalControlMode`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `2`

신호 정지 상태 처리를 선택합니다. 0: 사용 안 함, 1: 정지 판단만, 2: 정지와 출발 판단. 모델 예측과 유효한 내비 신호를 기존 조건에 따라 활용하며, 티맵 UI 표시 여부 자체를 제어하는 설정은 아닙니다.

사용처: [carcontroller.py](../../../opendbc_repo/opendbc/car/hyundai/carcontroller.py), [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

### 신호 정지 거리 보정(m) · `SignalStopDistanceOffsetM`

범위: -10 ~ 10 (단계 0.1) / 코드 기본값: `0.0`

모델 신호 정지 위치에 더하는 거리 보정입니다(단위 m). 양수는 목표를 더 앞쪽으로 보내 더 진행하게 하고, 음수는 목표를 가까이 당겨 일찍 멈추게 합니다. 주행 중 MPC 정지 장애물 위치에 적용하며 실제 정차 간격을 보장하는 값은 아닙니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py)

## Curv/Safety/Nav

### 차선 인식 모드 적용 속도 · `LaneModeMinSpeedKph`

범위: 0 ~ 200 (단계 2) / 코드 기본값: `20`

레인모드 요청에 사용하는 차량 속도 기준입니다(단위 km/h). 0은 기본 모델모드이며 양수는 기준 +5km/h 이상에서 레인모드, -5km/h 미만에서 모델모드를 요청합니다. 근거리 앞차·커브·모델 상태가 최종 모드를 바꿀 수 있습니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py), [lateral_planner.py](../../../openpilot/selfdrive/controls/lib/lateral_planner.py)

### 예측 커브 속도별 레인모드 기준 · `LaneModeCurveThresholdKph`

범위: 0 ~ 200 (단계 5) / 코드 기본값: `60`

레인모드 전환에 사용하는 예측 커브 속도 기준입니다(단위 km/h). 현재 차량 속도 기준이 아닙니다. 예측값과 기준의 ±5km/h 차이 및 약 0.5초 확인을 사용하며, 다른 모델모드 우선 조건이 있으면 전환하지 않을 수 있습니다.

사용처: [controlsd.py](../../../openpilot/selfdrive/controls/controlsd.py)

### 차선 변경 토크 요구 방식 · `LaneChangeRequiresTorque`

범위: -1 ~ 1 (단계 1) / 코드 기본값: `0`

차선변경 요청 시 운전자 핸들 입력 조건입니다. -1: 차선변경 비활성, 0: 토크 입력 없이 기존 조건으로 시작, 1: 방향에 맞는 운전자 토크 필요. 차선·도로경계·주변 차량 등의 제한은 별도로 적용됩니다.

사용처: [desire_helper.py](../../../openpilot/selfdrive/controls/lib/desire_helper.py)

### 차선 변경 대기 시간 · `LaneChangeDelaySec`

범위: 0 ~ 10 (단계 0.5) / 코드 기본값: `0.5`

차선변경 준비 상태에서 시작을 기다리는 시간입니다(단위 초). 0은 추가 대기 없음입니다. 시간이 지나도 토크·차선·사각지대 등의 조건이 충족되지 않으면 차선변경을 시작하지 않습니다.

사용처: [desire_helper.py](../../../openpilot/selfdrive/controls/lib/desire_helper.py), [selfdrived.py](../../../openpilot/selfdrive/selfdrived/selfdrived.py)

### BSD 기반 차선 변경 제한 · `LaneChangeBlindspotPolicy`

범위: -1 ~ 1 (단계 1) / 코드 기본값: `0`

순정 사각지대 신호 처리 방식입니다. -1은 BSD 신호를 무시합니다. 0은 BSD 감지 시 기다리지만 운전자 토크로 시작할 수 있고, 1은 BSD 대기 중 운전자 토크로 시작하는 것도 막습니다. 차선·도로경계·객체 등 다른 조건은 별도로 유지됩니다.

사용처: [desire_helper.py](../../../openpilot/selfdrive/controls/lib/desire_helper.py)

### 곡선 감속 최소 속도 · `CurveSpeedFloorKph`

범위: 30 ~ 200 (단계 5) / 코드 기본값: `40`

커브 감속 계산의 속도 하한입니다(단위 km/h). 커브 때문에 계산하는 제한 속도만 이 값 아래로 낮추지 않습니다. 앞차·신호·카메라 등 다른 이유로 차량은 더 느리게 주행하거나 정지할 수 있습니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 곡선 자동 감속 비율 · `VisionCurveSensitivityPct`

범위: 50 ~ 300 (단계 1) / 코드 기본값: `80`

모델의 예상 회전율에 곱하는 커브 민감도입니다(단위 %, 100이 기준). 크게 하면 같은 궤적을 더 강한 커브로 계산해 허용 속도가 낮아지는 방향으로 작용합니다. 실제 제한에는 커브 속도 하한과 다른 감속 조건도 적용합니다.

사용처: [nav_service.py](../../../openpilot/selfdrive/kisapilot/nav_service.py)

### 도로 제한 속도 오프셋 · `RoadLimitOffsetKph`

범위: -1 ~ 100 (단계 1) / 코드 기본값: `10`

도로 제한 속도에 더하는 설정 속도 오프셋입니다(단위 km/h). 0은 제한 속도 그대로, 10은 +10km/h입니다. -1은 가산 오프셋을 사용하지 않으며 도로속도 동기화에서는 안전 속도 비율로 대체하므로 자동 연동 전체를 끄는 값은 아닙니다. 카메라·방지턱·앞차 감속은 따로 계산합니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py), [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 도로 제한 속도 자동 보정 비율 · `SpeedLimitSetpointPolicy`

범위: -1 ~ 100 (단계 1) / 코드 기본값: `-1`

도로 제한 속도 변경 시 설정 속도 처리 방식입니다. -1은 새 도로속도+오프셋으로 동기화하며, 오프셋이 -1이면 안전 속도 비율을 사용합니다. 0은 이 보정 없음입니다. 양수는 제한 속도가 낮아질 때 기존 설정과 새 도로속도를 해당 비율(%)로 혼합하며, 앞차 기반 자동 속도 상승 기능의 조건도 충족해야 적용됩니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py)

### 과속 카메라 감속 완료 시점 · `EnforcementArrivalMarginSec`

범위: 3 ~ 20 (단계 1) / 코드 기본값: `8`

과속카메라 목표 속도를 미리 맞추기 위한 시간 여유입니다(단위 초). 차량 속도와 곱해 거리로 환산하므로 크게 하면 카메라보다 더 앞에서 감속을 완료하는 방향으로 작용합니다. 방지턱은 별도의 접근 시간과 감속 계산을 사용합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 내비 기반 감속 제어 모드 · `NavSpeedControlScope`

범위: 0 ~ 3 (단계 1) / 코드 기본값: `2`

내비 기반 속도 제한의 대상입니다. 0: 끄기, 1: 과속카메라, 2: 카메라와 방지턱, 3: 이동식 단속까지 포함. 유효한 위치·거리·제한 속도 정보가 있어야 해당 감속을 적용합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 과속 카메라 감속률 · `NavSpeedDecelMps2`

범위: 0.1 ~ 20 (단계 0.1) / 코드 기본값: `1.2`

내비 감속 곡선 계산에 사용하는 감속도 크기입니다(단위 m/s²). 작으면 더 멀리서 완만하게, 크면 더 가까이서 강하게 감속하는 곡선을 계산합니다. 방지턱은 이 값에 별도 보정·한계를 적용하므로 카메라와 동일한 곡선이 아닙니다.

사용처: [driving_policy.py](../../../openpilot/selfdrive/kisapilot/driving_policy.py), [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 과속 카메라 안전 계수 · `EnforcementTargetPct`

범위: 80 ~ 120 (단계 1) / 코드 기본값: `102`

단속 제한 속도에 곱하는 목표 속도 비율입니다(단위 %). 100은 제한 속도 그대로, 102는 제한 속도의 1.02배입니다. 감속 거리 여유나 센서 신뢰도를 뜻하는 값은 아닙니다.

사용처: [cruise.py](../../../openpilot/selfdrive/car/cruise.py), [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 방지턱 접근 시간 거리 · `BumpArrivalMarginSec`

범위: 1 ~ 50 (단계 1) / 코드 기본값: `2`

방지턱 목표 속도에 도달할 시간 여유를 계산하는 값입니다(단위 초). 거리로 환산하고 반응 지연·감속 한계와 함께 사용합니다. 크게 하면 더 앞에서 속도를 맞추는 방향이며 단순 감속 시작 타이머는 아닙니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 방지턱 통과 속도 · `BumpTargetKph`

범위: 10 ~ 100 (단계 5) / 코드 기본값: `30`

내비 방지턱 감속의 기본 통과 목표 속도입니다(단위 km/h). 학습이 켜져 있고 해당 위치에 기록이 있으면 위치별 학습 목표가 사용될 수 있습니다. 앞차 등 다른 제약이 더 낮으면 그 제약이 우선합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 방지턱 충격 학습 · `BumpLearningEnabled`

범위: 켜기/끄기 / 코드 기본값: `1`

방향별 방지턱 위치·통과 속도·충격을 로컬에 저장해 다음 접근 목표를 조절합니다. 큰 충격이나 수동 감속에는 목표를 낮추고, 조건을 만족한 반복적인 부드러운 통과·가속 페달 개입은 증속 후보로 사용합니다. 영상 모델을 실시간 재학습하는 기능은 아닙니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 학습 방지턱 접근 알림음 · `LearnedBumpAudioEnabled`

범위: 켜기/끄기 / 코드 기본값: `1`

학습된 위치에 접근할 때 알림음을 허용합니다. 학습 기록 저장·목표 속도 조절을 켜고 끄는 설정은 아니며 BumpLearningEnabled와 별도로 소리만 제어합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 방지턱 학습 최대 증속 · `BumpLearningMaxIncreaseKph`

범위: 0 ~ 10 (단계 5) / 코드 기본값: `10`

기본 방지턱 속도보다 학습으로 높일 수 있는 최대 증가량입니다(단위 km/h). 0은 기본 속도 초과 증속을 막습니다. 부드러운 통과 3회 또는 조건을 만족한 독립적인 페달 개입 3회 이상을 확인하며, 위치별 수동 상한과 학습 속도 상한 40km/h도 적용합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 내비 카운트다운 안내 범위 · `NavCountdownScope`

범위: 0 ~ 2 (단계 1) / 코드 기본값: `0`

내비 접근 카운트다운 이벤트의 범위입니다. 0: 끄기, 1: TBT·단속 안내(방지턱 제외), 2: 방지턱도 포함. 속도 제한 제어의 활성 여부와는 별도이며 실제 표시·음성은 이벤트 처리에 따릅니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 회전 속도 제어 모드 · `TurnSpeedControlMode`

범위: 0 ~ 3 (단계 1) / 코드 기본값: `1`

커브 속도 제한에 사용할 입력입니다. 0: 끄기, 1: 비전 궤적, 2: 비전과 지도 경로, 3: 지도 경로. 경로·모델 정보가 유효해야 적용하며 TBT 회전 속도 제어와는 별도 설정입니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 지도 회전 속도 계수 · `MapCurveTargetPct`

범위: 50 ~ 300 (단계 5) / 코드 기본값: `90`

지도 경로에서 계산한 커브 속도에 곱하는 비율입니다(단위 %, 100이 기준). 크게 하면 지도 기반 허용 속도가 높아집니다. 지도 경로 기반 모드를 사용하고 경로 정보가 있을 때 반영합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 모델 회전 속도 예측 시간(초) · `VisionTurnLookaheadSec`

범위: 0 ~ 8 (단계 0.1) / 코드 기본값: `0.0`

모델의 미래 속도를 참고할 예측 시점입니다(단위 초). 0은 이 모델 회전 속도 제한을 사용하지 않습니다. 양수는 해당 시점의 모델 속도를 보간해 사용하며 속도 배율이 아닙니다.

사용처: [desire_helper.py](../../../openpilot/selfdrive/controls/lib/desire_helper.py)

### TBT 자동 방향지시등 (실험) · `NavTurnSignalExperiment`

범위: 켜기/끄기 / 코드 기본값: `0`

아이오닉 5 PE에서 내비 TBT 좌·우회전 정보로 SPAS 방향지시등 자동 점등을 시험합니다. 차선변경·추월 자동 조향 설정과는 별도입니다. 처음 활성화한 뒤 C4를 재시작해야 차량 송신 구성이 반영되며, 끄기는 실행 중에도 반영됩니다.

사용처: [interface.py](../../../opendbc_repo/opendbc/car/hyundai/interface.py), [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 주행 중 방향지시등 시험 · `MovingBlinkerRoadTest`

범위: 켜기/끄기 / 코드 기본값: `0`

지원 차량의 이동 중 기사에이전트 버튼 시험과 추월 후보 점등 시험의 송신을 허용합니다. TBT 자동 방향지시등 실험과 함께 켜고 처음에는 C4를 재시작해야 합니다. 이 설정 자체가 자동 차선변경을 실행하지는 않습니다.

사용처: [interface.py](../../../opendbc_repo/opendbc/car/hyundai/interface.py), [kisa_agent.py](../../../openpilot/selfdrive/kisapilot/kisa_agent.py)

### 추월 후보 깜빡이 시험 · `OvertakeBlinkerSignalTest`

범위: 켜기/끄기 / 코드 기본값: `0`

현재 KYD의 추월 후보 점등 시험입니다. 후보 조건을 확인하면 왼쪽 깜빡이를 최대 3초 켜며 자동 조향·추월 가속은 하지 않습니다. 실제 점등 확인 뒤 BLINKER TEST와 음성을 안내합니다. TBT·이동 중 시험 설정 및 최초 재시작이 필요하며, 이 스위치는 약 0.5초마다 확인합니다. 운전자 개입 시 중단하고 같은 앞차에는 반복하지 않습니다.

사용처: [plannerd.py](../../../openpilot/selfdrive/controls/plannerd.py)

### 자동 회전 제어 모드 · `NavTurnAssistMode`

범위: 0 ~ 3 (단계 1) / 코드 기본값: `0`

TBT 회전 요청과 감속 사용 범위입니다. 0: 끄기, 1: 횡방향 회전 요청, 2: 횡방향 요청과 속도 제어, 3: 속도 제어만. 횡방향은 기존 desire·차선·운전자 개입 조건을 통과한 요청이며 교차로 회전 완료를 보장하지 않습니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 자동 회전 목표 속도 · `NavTurnTargetKph`

범위: 0 ~ 100 (단계 5) / 코드 기본값: `20`

TBT 좌·우회전의 기본 목표 속도입니다(단위 km/h). 유효한 회전 진행각이 약 45도 이상이고 커브 조건이 허용하면 해당 회전의 TBT 속도 제한을 최대 9km/h까지 점진 완화합니다. 원래 설정 속도·다음 회전·앞차·신호·단속 등 다른 제한은 유지합니다. AutoTurnControl의 속도 제어 모드에서 사용합니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### TBT 목표속도 도달 여유(초) · `NavTurnArrivalMarginSec`

범위: 0 ~ 30 (단계 1) / 코드 기본값: `6`

TBT 감속 목표를 미리 맞출 거리의 시간 계수입니다(단위 초). 회전 목표 속도 또는 분기 속도와 곱해 거리로 환산하고 남은 거리에서 뺍니다. 크게 하면 더 앞에서 목표 속도에 맞추는 방향이며 깜빡이 해제 타이머는 아닙니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

### 자동 지도 전환 · `NavTurnMapDisplay`

범위: 0 ~ 1 (단계 1) / 코드 기본값: `0`

TBT 회전 준비·해제 상태에 맞춰 연결 앱에 MAP/ROAD 화면 전환 명령을 보냅니다. 앱이 해당 명령을 지원해야 동작하며 차량 경로나 조향 방식을 바꾸는 설정은 아닙니다.

사용처: [nav_guidance.py](../../../openpilot/selfdrive/kisapilot/nav_guidance.py)

## DEV

### 스마트 프리빌드 사용 · `PutPrebuiltOn`

범위: 켜기/끄기 / 코드 기본값: `0`

부팅 스크립트에서 검증된 프리빌드 캐시 재사용을 허용합니다. 켜면 매번 전체 빌드하는 시간을 줄일 수 있고, 끄면 일반 소스 빌드 절차를 사용합니다. 실행 중 주행 계산에는 영향이 없으며 다음 부팅부터 적용합니다.

사용처: [launch_chffrplus.sh](../../../launch_chffrplus.sh)

### 주행 로그 기록 활성화 · `KisaEnableLogger`

범위: 켜기/끄기 / 코드 기본값: `0`

loggerd의 로컬 주행 로그 기록을 허용합니다. 꺼져 있으면 이후 분석에 필요한 로그가 저장되지 않을 수 있습니다. 서버 업로드 허용 여부를 정하는 스위치는 아니며 다른 녹화·업로드 기능은 각각의 설정을 따릅니다.

사용처: [process_config.py](../../../openpilot/system/manager/process_config.py)
