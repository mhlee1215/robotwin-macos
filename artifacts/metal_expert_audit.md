# Metal 전문가 실행 검사

50개 태스크를 동일 코드, seed 0으로 각 1회 실행했다. 다중 seed 논문 벤치마크 결과가 아니다.

27개 성공, 21개 경로 계획 실패, 1개 동작 후 작업 실패, 1개 불안정 초기 상태 거부. 실행 코드 예외와 timeout은 0개.

이전 CPU 개발 검사에서 실패했던 adjust_bottle, place_container_plate가 성공했다. 반대로 stamp_seal은 회귀했고 현재 CPU 재실행에서는 성공했다. 이전 CPU 전체 검사는 여러 개발 버전이 섞였으므로 집계 성공률의 엄밀한 비교 실험은 아니다.

stamp_seal의 실패 상태/목표에 CPU IK 64회와 Metal IK 2,048개 후보(400회 반복)를 적용했지만 유효 해를 찾지 못했다. 앞선 이동·잡기 상태 차이의 영향으로 추정되며, 정확한 원인은 미확정이다. 성공 판정이나 충돌 기준을 완화하지 않았다. 진단: [stamp_diagnostic/result.json](stamp_diagnostic/result.json). 대용량 `failed_ik.npz`는 이 저장소에 포함하지 않았다.

pick_diverse_bottles seed 0 거부는 그대로 유지했다. 별도 seed 1에서는 Metal 전문가 동작까지 성공했다 (`metal_diverse_expert_seed1/result.json`). 이 재시도를 seed 0의 성공으로 합산하지 않는다.

| 태스크 | 결과 |
|---|---|
| adjust_bottle | success |
| beat_block_hammer | planning_failure |
| blocks_ranking_rgb | planning_failure |
| blocks_ranking_size | planning_failure |
| click_alarmclock | planning_failure |
| click_bell | success |
| dump_bin_bigbin | success |
| grab_roller | planning_failure |
| handover_block | success |
| handover_mic | planning_failure |
| hanging_mug | success |
| lift_pot | success |
| move_can_pot | success |
| move_pillbottle_pad | success |
| move_playingcard_away | success |
| move_stapler_pad | success |
| open_laptop | planning_failure |
| open_microwave | planning_failure |
| pick_diverse_bottles | unstable_seed |
| pick_dual_bottles | success |
| place_a2b_left | planning_failure |
| place_a2b_right | planning_failure |
| place_bread_basket | success |
| place_bread_skillet | success |
| place_burger_fries | planning_failure |
| place_can_basket | success |
| place_cans_plasticbox | success |
| place_container_plate | success |
| place_dual_shoes | success |
| place_empty_cup | success |
| place_fan | success |
| place_mouse_pad | success |
| place_object_basket | task_failure |
| place_object_scale | planning_failure |
| place_object_stand | success |
| place_phone_stand | planning_failure |
| place_shoe | success |
| press_stapler | planning_failure |
| put_bottles_dustbin | success |
| put_object_cabinet | planning_failure |
| rotate_qrcode | success |
| scan_object | planning_failure |
| shake_bottle | planning_failure |
| shake_bottle_horizontally | planning_failure |
| stack_blocks_three | success |
| stack_blocks_two | success |
| stack_bowls_three | planning_failure |
| stack_bowls_two | planning_failure |
| stamp_seal | planning_failure |
| turn_switch | success |
