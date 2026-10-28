## 2026-10-28 test6 보완

- 동일 앞차의 중앙 제동등이 기존 비점등 기준으로 돌아왔으면 좌우 밝기 관측의 이전 ON을 표시하지 않습니다. 기존 좌우 관측이 OFF를 확인하거나 새로운 중앙 점등이 확인되면 해제합니다. 가림/기준 불명은 OFF로 단정하지 않습니다.
- 중앙 LED 스트립 관측에 이웃 한 픽셀을 포함해 샘플링 위치 변화만으로 점등/소등 근거가 생기는 것을 줄입니다.
- 얇고 중앙에 정렬된 가로형 후미등의 양 끝을 위치 후보로 사용할 수 있습니다. 이 가상 좌우 쌍으로 밝기 기준을 만들거나 ON/OFF를 확정하지 않습니다. 독립적인 중앙등 변화가 없으면 UNKNOWN을 유지합니다.
- 320 모델의 단일 스레드 준비 측정이 700ms를 넘을 때에만 배경 스레드 2개 설정을 추가 측정합니다. FP32/INT8·스레드 설정 중 실제 측정이 5% 이상 빠른 설정을 선택하고, ORT 대기 중 spin을 끕니다. 합산 CPU 시간을 이용해 실행 간격을 제한합니다. 1초 추론/결과 신선도 제한은 그대로입니다.
- 테스트는 모의 ORT·PC 픽셀 처리 검사입니다. 실제 C4에서 병렬 설정이 1초 안에 실행되는지는 아직 확인하지 못했습니다. 제한을 초과하면 기존과 같이 재시도/비활성화합니다.
- test6 구간0/4/5/6/9의 2749개 유효 ROI를 약10Hz로 재생했습니다. 구간4→5→6 사이 관측/보정 정보를 이어 사용하고 앞차/보정 변경 및 신선도 조건을 반영했습니다. 선택 구간의 ON/OFF 결과는 기존과 같고, 구간9의 가로형 등 위치 후보는 복구됐습니다. UNKNOWN을 맞은 ON으로 집계하지 않습니다.
- 실기 구간5 출발 때의 ON 오검출은 이 PC 재생에서 재현되지 않았습니다. 중앙등 소등·차체 어두워짐·좌우 ON 잔류를 구성한 회귀 검사로 차단을 확인했으며 해당 실기 장면의 완전한 해결을 주장하지 않습니다.
- 차량 제어/감속에는 연결하지 않은 표시용 기능입니다. 수동 정답 저장 화면은 제거된 상태입니다. 아래 내용은 이전 구현과 검사의 기록입니다.

# C4 선행차 제동등 시험 표시 — realdata 검증 반영

D:\9999.KYD\realdata 아래 압축 해제된 fcamera.hevc 34개(40800프레임, 약 34분)를 전부 디코딩해 이전/최종 코드로 검사했습니다. 각 구간은 1분이며 test1 10개, test2 13개, test3 6개, test4 5개입니다. 전 프레임을 사람이 정답으로 라벨링한 검증은 아닙니다. 약 10Hz의 UI 호출을 PC에서 모사하고 10초 간격 대표 장면과 점등/오류 후보를 영상으로 대조했습니다. 34개 로그 모두 os04c10 센서를 확인했습니다.

## 적용 내용

- 가까운 앞차의 지면 탐색 범위가 화면 아래로 조금 나가도, 75% 이상 남아 있으면 아래쪽만 잘라 탐색합니다. 좌우 경계를 벗어난 대상을 다른 차량 쪽으로 옮겨 탐색하지 않습니다.
- 같은 앞차에서 직전 등 배치와 가까운 후보를 우선해 반사판 등으로 옮겨가는 현상을 줄입니다.
- 짧은 등 검출 누락 시 ON/위치 표시는 즉시 지우되, 기준은 마지막 정상 관측부터 최대 0.4초만 보존합니다. 오래 놓치거나 대상/등 배치가 바뀌면 기준을 버립니다.
- 밝기 기준은 최근 관측의 하위 백분위로 갱신합니다. 최근 프레임 대비 점등 시작 변화도 필요하므로, 거리·조명이 서서히 바뀌는 것만으로 새 ON을 만들지 않도록 합니다.
- 중간 검사에서 화물차의 작은 노란 표식을 보조제동등으로 오인하고, 차체만 어두워진 장면을 ON으로 잡았습니다. 붉은 색 기준과 보조등 최소 크기를 보강하고, 정규화 비율 외에 등 자체의 원시 밝기 증가도 확인합니다. 해당 화물차 38.9/47.1초의 잘못된 ON은 최종 검사에서 제거됐습니다.

## 화면 및 로그

회색 모서리는 leadOne의 투영 탐색 범위이며 차량 검출 경계가 아닙니다. 작은 테두리는 등 후보이고 주황색 BRAKE ON?는 점등 후보입니다. LAMPS OFF?도 비점등 후보이며, 초기/불확실 상태는 LAMPS UNKNOWN입니다. 처음부터 제동등이 켜진 차량을 곧바로 OFF로 학습하지 않습니다.

대상 교체, 영상/메시지 지연, 보정 무효 시 이전 표시를 지우고 기준을 초기화합니다. radarTrackId가 없는 비전 선행차에서 같은 차량인지 완벽히 보장되지는 않습니다. leadOne 주변에서만 탐색하지만 그 안에 겹친 다른 물체를 혼동할 수 있습니다. 차종/등을 학습한 신경망 검출기가 아닌 휴리스틱입니다.

cloudlog에 lead_brake_lights_ui 이벤트로 state, reason, frame_id, d_rel, v_rel, gains, processing_ms 등을 기록합니다. 정상 logmessaged/loggerd가 실행되면 주행 로그에서 확인할 수 있습니다. 별도 제어 메시지는 없습니다.

기본값은 시험 표시 활성화이며 UI 프로세스의 LEAD_BRAKE_LIGHTS_UI=0 환경변수로 끌 수 있습니다. 최대 192픽셀 영역을 UI에서 복사하고 단일 작업 스레드에서 최대 10Hz 처리하며, 대기 작업을 쌓지 않습니다. 선택 기능의 예외는 표시를 지우고 30초 뒤 재시도합니다.

## 결과와 한계

UI 표본 20258개 중 이전 숨김은 12987개, 최종 숨김은 5555개입니다. 이 감소는 탐색/표시 가능 범위의 변화이며 인식률 향상 수치가 아닙니다. 최종 UNKNOWN은 14245개로 여전히 많습니다. 제동등이 처음부터 켜진 상태, 가로로 연결된 등, 작은/가려진 등, 화물차 등에서 미검출과 판정 불확실성이 남습니다. ON이 줄었다고 정밀도가 검증된 것도 아닙니다.

22개 테스트 통과: 초기 점등 UNKNOWN, 밝기 증가/감소, 단일 프레임/한쪽 등/전체 노출 변화, 그림자만 생김, 작은 노란 표식, 점등 근거 소실, 짧은/긴 누락, 느린 밝기 변화, 등 배치 변화, 제한적 화면 아래 절단, 잘못된 투영, NV12 패딩/복사, 지연 데이터, 대상 교체 후 이전 비동기 결과 폐기 등입니다. Python 문법과 패치 공백도 검사했습니다.

```powershell
python openpilot/selfdrive/ui/tests/test_brake_lights.py
```

이 결과는 PC의 로그/영상 재생입니다. 실제 C4의 VisionIPC/GPU UI를 실행하거나 지연·발열을 측정하지 않았습니다. 제어·감속 계획에는 연결하지 않았고 실제 C4에 배포·재시작하지 않았습니다. 다음 단계는 수동 정답 라벨을 통한 누락/오인식 측정, 차량/등 자동 검출 개선과 C4에서 표시 전용 실기 검증입니다.


## C4 mici 오른쪽 상태 패널

초록색 ConfidenceBall 대신 기존 60px 사이드 영역에 앞차 상태를 표시합니다. 카메라/티맵 화면 폭을 바꾸지 않습니다. 내비게이션 패널이 표시 중일 때는 기존과 같이 이 사이드 표시는 숨깁니다.

C4 한글 폰트 제약에 맞춰 화면 문구는 ASCII만 사용합니다: LEAD, BRAKE, ON? / OFF? / WAIT, 거리(m), CLOSING / AWAY / STEADY. 주황색은 점등 후보, 청록색은 비점등 후보, 회색은 판별 중입니다. 상대속도 ±0.3m/s 안에서는 STEADY로 표시하며, 접근은 위쪽 꺾쇠, 멀어짐은 아래쪽 꺾쇠로 구분합니다.

거리·상대속도는 신선한 radarState.leadOne에서 가져오고, 제동등 상태는 유효한 최신 관측(0.35초 이내)만 사용합니다. 앞차가 없거나 radarState가 오래되면 패널을 숨깁니다. 영상 판정이 없거나 만료돼도 앞차 데이터가 신선하면 회색 WAIT와 거리·접근 상태는 표시합니다. 영상 위의 긴 영문 상태 문구는 제거하고 등 위치 표시만 유지합니다.

패널 표시·폭 맞춤·ASCII·오래된 데이터·앞차 없음·사용 중지 등 패널 테스트 9개, 기존 인식/표시 연동 테스트 23개가 통과했습니다. C4 실기 글꼴/GPU 실행 확인은 별도입니다.

왼쪽 TPMS/차간 간격 표시 위에 중복으로 표시하던 앞차 거리 숫자(s.radarDRel)는 제거했습니다. 차간 간격 막대·타이어압·정지선/티맵 안내 거리는 유지합니다. 앞차 거리는 오른쪽 패널에만 표시합니다.


## C4 mici layout (2026-10-24)

The rightmost 60 logical pixels hold ego speed and the lead brake observation panel.
Speed remains visible without a lead and with navigation; navigation uses the space to its left.
Digits fit the strip, including three digits, and their visible top aligns with the signal/offset header.
The signal panel is at the upper right of the video, with a 20px corner inset.
Driver, car/TPMS, four fixed gap slots, and CAM/BUMP share x=46 in widget coordinates.
The old left lead-distance label and the floating roof gap bar are removed.
TPMS values are larger; AutoHold is shown as H inside the rear of the car.
CAM/BUMP replaces the lower steering wheel while fresh guidance exists. The nearest valid
remaining distance wins; BUMP wins ties. When BUMP clears, a remaining CAM becomes visible.
Critical steering instructions retain priority. Stale guidance falls back to the wheel.
All on-screen labels are ASCII. This display priority does not modify longitudinal control.
PC tests and drawing-call previews do not establish correct operation on C4 hardware.
