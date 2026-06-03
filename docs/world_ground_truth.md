# re540_final_map — Ground-Truth Reference

Dumped from `/gazebo/model_states` after `roslaunch nexus_4wd_mecanum_gazebo
re540_bringup.launch gui:=false`. Use this as the truth column when writing
`config/stores.yaml`, validating signboard observations, and scoring the
LLM agent's category inference in the report.

## Stores (8 entities, ground-truth labels)

| name (gazebo)        | x      | y      | z   | inferred category |
|----------------------|--------|--------|-----|-------------------|
| store_burger_red     | -1.251 | -1.000 | 0.2 | fast_food         |
| store_burger_yellow  |  1.251 | -1.000 | 0.2 | fast_food         |
| store_cafe_blue      | -0.749 | -0.700 | 0.2 | cafe              |
| store_cafe_yellow    |  1.000 | -2.010 | 0.2 | cafe              |
| store_phar_red       |  1.000 |  1.490 | 0.2 | pharmacy          |
| store_phar_white     |  0.749 | -1.300 | 0.2 | pharmacy          |
| store_store_blue     | -1.000 |  1.490 | 0.2 | convenience       |
| store_store_green    | -1.000 | -2.010 | 0.2 | convenience       |

## Signboards (16 entities, ~0.405 m tall)

Two facing rows around y ≈ ±0.629 and ±0.871 plus outer corridors at
±2.75. Each signboard bundle holds 2–3 AprilTags (see `tags.yaml`).

| name              | x      | y      |
|-------------------|--------|--------|
| signboard_white_01| -2.000 | -2.629 |
| signboard_white_02| -2.000 |  0.629 |
| signboard_white_03| -2.000 |  0.871 |
| signboard_white_04| -1.879 | -2.750 |
| signboard_white_05| -1.879 |  0.750 |
| signboard_white_06| -0.121 | -2.750 |
| signboard_white_07| -0.121 |  0.750 |
| signboard_white_08|  0.000 | -2.629 |
| signboard_white_09|  0.000 |  0.629 |
| signboard_white_10|  0.121 | -2.750 |
| signboard_white_11|  0.121 |  0.750 |
| signboard_white_12|  1.879 | -2.750 |
| signboard_white_13|  1.879 |  0.750 |
| signboard_white_14|  2.000 | -2.629 |
| signboard_white_15|  2.000 |  0.629 |
| signboard_white_16|  2.000 |  0.871 |

## How this relates to `Store coordinates.txt`

The TA-provided file has 8 (x, y) pairs without categories — the LLM agent
must infer categories from signboards. Mapping txt rows to gazebo stores
(by nearest neighbor):

```
(-1.25, -1)    → store_burger_red       (fast_food)
(-1, -2)       → store_store_green      (convenience, |dy|=0.01)
(-1, -1.5)     → store_store_green      (convenience, |dy|=0.51 — txt typo?)
(-0.75, -0.7)  → store_cafe_blue        (cafe)
(0.75, -1.3)   → store_phar_white       (pharmacy)
(1, -2)        → store_cafe_yellow      (cafe, |dy|=0.01)
(1, 1.5)       → store_phar_red         (pharmacy, |dy|=0.01)
(1.25, -1)    → store_burger_yellow    (fast_food)
```

The third row `(-1, -1.5)` collides with row 2 against `store_store_green`
(dy=0.5). Possibly a typo for `(-1, -2.01)`-the row 2 duplicate, OR the txt
encodes ONE store that has shifted between draft and final map. Will flag
in `config/stores.yaml` as ambiguous, drive to the nearest gazebo store
when invoked.
