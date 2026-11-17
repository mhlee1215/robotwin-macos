# Metal expert-script audit

All 50 tasks were run once at seed 0 with the same implementation. This is not a multiseed paper benchmark.

Results: 27 successes, 21 planning failures, one task failure after execution, and one unstable-initial-state rejection. There were no runtime exceptions or timeouts.

`adjust_bottle` and `place_container_plate` succeeded after failing in the earlier CPU development audit. `stamp_seal` regressed and still succeeds when rerun with the current CPU implementation. The earlier full CPU audit mixed development revisions, so its aggregate success rate is not a controlled comparison.

At the failed `stamp_seal` state and target, 64 CPU IK attempts and 2,048 Metal candidates with 400 iterations found no valid solution. Earlier motion or grasp-state differences may contribute; the exact cause is unconfirmed. Success and collision criteria were not relaxed. See the [diagnostic record](stamp_diagnostic/result.json). The large `failed_ik.npz` file is not included in this repository.

The rejection of `pick_diverse_bottles` at seed 0 is retained. A separate Metal expert run at seed 1 succeeded; see the [separate record](metal_diverse_expert_seed1/result.json). This retry is not counted as a seed 0 success.

| Task | Result |
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
