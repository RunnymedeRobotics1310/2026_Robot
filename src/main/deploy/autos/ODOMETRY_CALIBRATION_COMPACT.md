# Compact odometry calibration (limited floor space)

These autos are designed to fit in a small area while still giving useful odometry scale signals.

## Files

- `odometry_cal_straight_compact.json`
  - Use for wheel distance scale (`WHEEL_DISTANCE_SCALE`).
- `odometry_cal_spin_compact.json`
  - Use for turn-in-place drift minimization (geometry scale first pass).
- `odometry_cal_turn_compact.json`
  - Use for mixed translation + rotation final validation/tuning.

## Preparation

- Disable vision pose fusion for calibration runs.
- Start from the same marked pose and heading each run.
- Run each test at least 3 times and average results.
- Use blue side first for consistency (alliance heading offsets can still be tested later).

## 1) Tune wheel distance scale

Run `odometry_cal_straight_compact`.

This template is one-way (single 2.0 m forward leg). Reposition to the start mark between runs.

Measure:
- `actual_leg_m`: tape-measured physical distance of each 2.0 m leg (typically measure the forward turn-point distance from start).
- `commanded_leg_m`: 2.0 m in this template.
- Average across multiple legs/runs to reduce noise.

Update:

```
WHEEL_DISTANCE_SCALE_new = WHEEL_DISTANCE_SCALE_old * (actual_leg_m / commanded_leg_m)
```

If you aggregate multiple legs:

```
WHEEL_DISTANCE_SCALE_new = WHEEL_DISTANCE_SCALE_old * (sum(actual_leg_m) / sum(commanded_leg_m))
```

Apply in `Constants.Swerve.WHEEL_DISTANCE_SCALE`, redeploy, re-run until each straight leg is close to target.

## 2) Tune geometry scales in place

Run `odometry_cal_spin_compact`.

Goal:
- Final pose should return near start with minimal XY drift.

Update strategy:
- Keep `ODOMETRY_WHEEL_BASE_SCALE` and `ODOMETRY_TRACK_WIDTH_SCALE` equal at first.
- Try `0.98`, `1.00`, `1.02` and pick the value with the smallest final XY drift.
- Then refine around the best value in smaller steps (for example `+/-0.005`).

## 3) Validate with mixed-turn path

Run `odometry_cal_turn_compact`.

Goal:
- End close to start pose (both translation and heading), with consistent behavior CW/CCW.

If errors are still directional:
- Adjust `ODOMETRY_TRACK_WIDTH_SCALE` slightly for forward-turn coupling issues.
- Adjust `ODOMETRY_WHEEL_BASE_SCALE` slightly for strafe-turn coupling issues.
- Change one constant at a time, keep notes, and repeat.

## Practical acceptance target

- Straight-line: <= 1 cm per 2 m segment.
- Mixed-turn compact loop: <= 5 cm final translation error after the full sequence.
