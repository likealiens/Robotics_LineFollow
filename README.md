# Line Following Robot (TRIK Studio)

A two-sensor line follower built from TRIK Studio nodes. The program is in `Line_FollowingRobot.qrs`.

## Program structure

```
Initial Node -> Expression (init) -> Expression (controller) -> Motors M3 -> Motors M4 -> Timer (1 ms) --+
                                            ^                                                            |
                                            +------------------------------------------------------------+
```

| Block | Content |
|---|---|
| Expression (init, runs once) | stores the starting sensor readings and sets the gains |
| Expression (controller, every cycle) | computes the error and the steering correction `u` |
| Motors Forward, port `M3` | power `30 + u` |
| Motors Forward, port `M4` | power `30 - u` |
| Timer | 1 ms delay, then back to the controller Expression |

## How the controller works

The robot has two light sensors: **A4 (left)** and **A3 (right)**. It compares what they see to find out how far it is from the line, and steers to cancel that offset.

### Reference values

The first Expression stores the sensor readings at the moment the program starts:

```
left  = sensorA4;
right = sensorA3;
```

Place the robot with both sensors on the line before pressing Run. Every later reading is measured relative to these two values, so a small difference between the two sensors does not fool the controller, and the program does not depend on fixed brightness thresholds.

### Error

```
err = (sensorA4 - left) - (sensorA3 - right)
```

Zero means the robot is centred. Positive means the left sensor has moved onto brighter floor compared with the start (the line is to the right). Negative means the opposite.

### Controller

```
err = (sensorA4-left) - (sensorA3-right);
p = K_p * err;
integral = integral + err * DT;
i = K_i * integral;
d = K_d * (err - last_error)/10;
u = p + i + d;
last_error = err;
```

### Motors

```
M3 = 30 + u
M4 = 30 - u
```

One wheel speeds up while the other slows down, so the robot turns toward the line. `30` is the constant forward base speed.

## The three terms

- **`K_p` (proportional): react to the mistake now.** Steering = `K_p * err`. The further off the line, the harder it turns. Too small and the robot is lazy and loses the line in curves. Too big and it zigzags.
- **`K_d` (derivative): react to how fast the mistake is changing.** It damps the oscillation caused by steering lag. If the error is shrinking quickly, the robot eases off before overshooting. It can make the robot jumpy if the sensors are noisy.
- **`K_i` (integral): react to a mistake that keeps happening.** `integral` is a plain running sum of `err * DT`. It corrects a persistent offset, for example one motor being slightly stronger than the other. On a curve the error is legitimately non-zero, so the sum keeps growing and the robot can overshoot the exit of the curve. There is no anti-windup clamp, so keep `K_i` small.

## Gains

The file is saved with these values in the init Expression:

| `K_p` | `K_i` | `K_d` | `DT` | Base speed |
|---:|---:|---:|---:|---:|
| 1 | 0 | 15 | 0.1 | 30 |

With `K_i = 0` this is a **PD controller**. The other controller types come from changing only the gains in the init Expression:

| Controller | `K_p` | `K_i` | `K_d` |
|---|---|---|---|
| P | 1 | 0 | 0 |
| PI | 1 | small, > 0 | 0 |
| PD (as saved) | 1 | 0 | 15 |
| PID | 1 | small, > 0 | 15 |

## Notes and limitations

- `DT = 0.1` and the `/10` in the derivative are fixed constants, not a measured loop time. The Timer is set to 1 ms, but the real cycle time depends on how fast the interpreter runs. `K_i` and `K_d` are therefore per-cycle gains that were tuned by hand.
- The program has no stop condition. It runs until it is stopped manually.
- Sensor ports and motor signs are specific to this robot. If it steers away from the line, swap `30 + u` and `30 - u`.
- If both sensors end up on white, the error is close to 0 and the robot drives straight. There is no line-recovery search.

## Files

- `Line_FollowingRobot.qrs`: the TRIK Studio program
- `README.md`: this document
