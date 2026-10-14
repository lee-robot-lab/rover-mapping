# 좌표·보정 참고 URDF

최신 매핑 기구 기준은 **`urdf/rocker_bogie_4wd_rover.urdf`**이며, 사용자가 카메라 하향 20°를 확인했다. [장착 좌표·사용법·검증 범위](../../../docs/experiments/2026-10-14-rover-extrinsics-update.md)를 따른다. `urdf/rover.urdf`는 과거 35° 기록용으로 보존한다.

이 폴더는 전체 rover_description ROS 패키지/mesh/simulation 배포본이 아니다. 최신 URDF의 메시를 표시하려면 별도 `rocker_bogie_4wd_rover` 패키지가 필요하다. 보정 계산은 메시를 읽지 않는다.
