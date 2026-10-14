# Flight Setup

Current source code is the authoritative configuration.
The `.ioc` mirrors the active pinout, timers, UART, I2C1 pins, and clock tree. GY-87 access now uses hardware `I2C1` on `PB6/PB7`, initialized manually in `Core/Src/main.c` so the flight build does not depend on generated HAL I2C source.

## Wiring

### Power

- LiPo `3S` -> power distribution -> `4 ESC`
- Only one ESC BEC `5V` line goes to `Blue Pill 5V` and `Receiver VCC`
- The other three ESC `5V` lines must be disconnected or insulated
- All grounds must be common: `Blue Pill`, `Receiver`, `ESC`, `GY-87`

### GY-87

- `3V3` -> `VCC`
- `GND` -> `GND`
- `PB6` -> `SCL` (`I2C1_SCL`)
- `PB7` -> `SDA` (`I2C1_SDA`)
- Make sure the bus has valid pull-ups. Most GY-87 boards already provide them.
- Current airframe orientation: `chip up`
- Roll and pitch are remapped in firmware for the installed board orientation, and yaw gyro feedback is inverted with `GYRO_Z_SIGN = -1.0f` to match the tested hardware response. Do not change this back to `1.0f` on the current frame because it causes strong clockwise yaw.
- Firmware now uses the onboard `HMC5883L` magnetometer over the same hardware `I2C1` bus. `BMP180` support is intentionally disabled in this build so the flight stack stays simpler and more stable until the later MS5611 upgrade.

### Receiver PWM

- `PA6` <- `CH1 Roll`
- `PA7` <- `CH2 Pitch`
- `PB0` <- `CH3 Throttle`
- `PB1` <- `CH4 Yaw`

### ESC Outputs

- `PA0` -> `ESC1` -> `M1 front-right`
- `PA1` -> `ESC2` -> `M2 front-left`
- `PA2` -> `ESC3` -> `M3 rear-left`
- `PA3` -> `ESC4` -> `M4 rear-right`

### Debug / Flash

- `PA9`  -> USB-TTL RX
- `PA10` -> USB-TTL TX
- `PA13` -> SWDIO
- `PA14` -> SWCLK
- Firmware expects a working `8 MHz HSE` crystal and runs the MCU at `72 MHz`

## Frame Layout

```text
           FRONT
      M2(CW)    M1(CCW)

      M3(CCW)   M4(CW)
```

## Control Mode

- Flight mode is fixed to `ANGLE`
- Arm: `Throttle low + yaw right hold 3s`
- Disarm: `Throttle low + yaw left hold 3s`
- When armed, low throttle keeps stabilization active instead of flattening all motors to the same idle value

## Safety

- Remove all propellers before first power-on
- Keep the frame still and level during IMU calibration
- Do not arm unless throttle is low
- While throttle is low, yaw stick is reserved for arm/disarm only and is forced out of flight control so arming cannot preload yaw PID or split the motors.
- If receiver signal is lost while airborne, the firmware now detects either missing PWM or the receiver's preset-loss PWM pattern, then enters a softer failsafe sequence: hold level longer, descend more gently, and hand control back smoothly if RF returns
- If the IMU fails repeatedly in flight, the firmware still performs an emergency disarm because stable control is no longer possible
- Yaw stick centered enables true gyro-integrated heading hold: `REF` is locked when heading hold engages and no longer follows `HDG` during yaw errors. The magnetometer corrects yaw estimate slowly and is rejected when the field strength, heading jump, tilt, roll/pitch rate, throttle change, or accel trust looks unhealthy.
- UART status log now includes `FM` failsafe mode, `RL` preset-loss detection, `FTH` failsafe throttle, `HDG` yaw estimate, `REF` heading-hold target, `HH` heading-hold state, `HA` heading-hold allowed state, `MU` magnetometer usable state, `MA` magnetometer correction allowed state, `MF` magnetic field strength, `ACC` accel norm, `TB` tilt-throttle boost, `TA` takeoff assist, `TD` throttle delta, `TR` throttle rate, `DYR` desired yaw rate, `GX/GY/GZ` gyro rates, `HE` limited heading error, `YD` dynamic yaw damping, `YT` throttle/takeoff yaw feed-forward, `YF` total yaw feed-forward, `YB` roll/pitch maneuver yaw-bias feed-forward, `YBL` learned yaw bias, `PID`, and mixer saturation fields.

## First Bring-up Checklist

1. Power the board without propellers
2. Verify UART log at `115200` on `USART1`
3. Check each receiver channel moves around `1000-2000 us`
4. Check arming only works with `Throttle low + yaw right hold 3s`
5. Immediately after arming and releasing yaw stick, confirm `CMD` yaw is `0`, `DYR=0`, `PID` yaw is near `0`, and the motors are not split into a strong yaw pair.
6. Confirm the frame lifts without immediately dropping toward one side
7. While armed at low throttle, tilt the frame by hand and confirm the mixer still reacts to hold attitude
8. Check motor order and motor direction one by one
9. With propellers removed and yaw stick centered, twist the frame clockwise by hand above yaw-control throttle and confirm the yaw mixer reacts against that motion. For the current motor layout, a clockwise twist should make the controller raise the counter-clockwise torque pair and reduce the clockwise motion; it must not accelerate the clockwise yaw.
10. Repeat step 9 with ESCs powered from the flight battery, still with propellers removed, because real ESC/motor power can reveal wiring, ground, and magnetic-noise issues that USB-only bench logs cannot show.
11. Mount GY-87 firmly and tell the firmware which edge faces front if not default
12. Test RF failsafe without propellers: arm, raise throttle a little, cut transmitter power, confirm `RL=1` and `FM=1/2` appear in the log instead of an immediate motor cut, then restore RF and confirm the log returns to normal mode
13. Check `MAG=1`, `MU=1`, `MC=1`, valid `HDG`, and changing `MX/MY/MZ` values appear in the UART log before prop-on hover tests
14. With motors spinning and props removed, confirm `MU` stays stable. If `MU` drops or `MH/MF` jumps badly, move the GY-87/compass wiring farther from ESC and battery leads before flying.
15. Roll and pitch the frame by hand without touching yaw. `HDG` should not jump sharply, `REF` should stay fixed while `HH=1`, `HA` should stay active above takeoff throttle, `MA` may drop during dirty compass conditions, and the motors should not accelerate clockwise yaw when roll/pitch corrections are active.

## Tuning Knobs In Code

Main settings are at the top of `Core/Src/main.c`:

- `SENSOR_ROTATION`
- `RC_ROLL_SIGN`
- `RC_PITCH_SIGN`
- `RC_YAW_SIGN`
- `RC_ROLL_CENTER_US`
- `RC_PITCH_CENTER_US`
- `RC_YAW_CENTER_US`
- `RC_THROTTLE_MIN_US`
- `RC_THROTTLE_MAX_US`
- `MAX_ANGLE_DEG`
- `SENSOR_LEVEL_ROLL_OFFSET_DEG`
- `SENSOR_LEVEL_PITCH_OFFSET_DEG`
- `HOVER_ROLL_TRIM_DEG`
- `HOVER_PITCH_TRIM_DEG`
- `ARM_HOLD_MS`
- `DISARM_HOLD_MS`
- `pid_roll`, `pid_pitch`, `pid_yaw`
- runtime RC center learning for roll/pitch while disarmed/throttle-low; yaw center stays fixed at `RC_YAW_CENTER_US`
- magnetometer health thresholds
- `MAG_ROTATION`, `MAG_X_SIGN`, `MAG_Y_SIGN`, `MAG_Z_SIGN`
- tilt-throttle compensation limits
- `MOTOR_HEADROOM_US`
- `YAW_CONTROL_MIN_THROTTLE`, `HEADING_HOLD_MIN_THROTTLE`, `HEADING_HOLD_RELEASE_MS`
- `YAW_DYNAMIC_DAMPING_KP`, `YAW_DYNAMIC_DAMPING_THR_KP`, `YAW_DYNAMIC_DAMPING_RP_KP`
- `YAW_THROTTLE_FF`, `YAW_THROTTLE_HIGH_FF`, `YAW_THROTTLE_DELTA_FF`, `YAW_TAKEOFF_BIAS_FF`
- `YAW_ROLL_COUPLING_FF`, `YAW_PITCH_COUPLING_FF`
- `YAW_RP_MANEUVER_BIAS_FF`, `YAW_RP_OUTPUT_BIAS_FF`, `YAW_RP_RATE_BIAS_FF`, `YAW_RP_BIAS_THR_GAIN`, `YAW_RP_BIAS_LIMIT`
- `YAW_AUTHORITY_FLOOR`, `YAW_MANEUVER_AUTHORITY_MIN`, `YAW_MIX_SIGN`
- `YAW_BIAS_LEARN_*`, `YAW_BIAS_DECAY`, `YAW_BIAS_LIMIT`
- `HEADING_HOLD_ERROR_LIMIT_DEG`
- `YAW_SAT_INTEGRATOR_DECAY`
- `TAKEOFF_ASSIST_*`
- `TAKEOFF_ROLL_TRIM_DEG`, `TAKEOFF_PITCH_TRIM_DEG`

If roll, pitch, or yaw moves the wrong way on bench, change the corresponding `RC_*_SIGN` or motor direction before flight.

Current firmware is pre-calibrated for the UART values measured from your radio:

- `roll center ~= 1539`
- `pitch center ~= 1525`
- `yaw center ~= 1508`
- `throttle min ~= 990`
- `throttle max ~= 2120`

Current `pid_roll` / `pid_pitch` values are kept stable, while the angle outer loop and stick filtering are tuned for slower, smoother roll/pitch response. This build reduces the center-stick brake boost and filters roll/pitch stick input more strongly so the frame is easier to place without twitchy corrections. `pid_yaw` keeps enough heading-hold authority, yaw damping, and integrator authority for the current frame's clockwise yaw bias. Heading hold now locks `REF` instead of letting it drift with `HDG`; only pilot yaw input, low throttle/yaw-control disable, arming state changes, failsafe, or sensor faults reset the reference. Dynamic yaw damping and throttle feed-forward counter clockwise torque bias, while roll/pitch maneuver feed-forward helps counter yaw that appears during roll or pitch corrections. A slow `YBL` learned-yaw-bias term now adapts residual torque when yaw stick is centered, throttle is in the flight range, and the mixer is not saturated.

If a consistent small yaw remains after confirming motor direction, tune in this order:

1. `YAW_THROTTLE_DELTA_FF` for yaw that appears only while moving the throttle stick.
2. `YAW_THROTTLE_FF` for yaw bias that grows with steady throttle.
3. `YAW_THROTTLE_HIGH_FF` for yaw that becomes stronger at high throttle.
4. `YAW_TAKEOFF_BIAS_FF` for yaw that appears during centered-stick liftoff.
5. `YAW_RP_MANEUVER_BIAS_FF` for clockwise yaw that appears during roll/pitch regardless of stick direction.
6. `YAW_RP_OUTPUT_BIAS_FF` if yaw appears when the controller is making strong roll/pitch corrections even with small stick input.
7. `YAW_RP_RATE_BIAS_FF` if yaw appears mainly while the frame is rotating quickly around roll/pitch.
8. `YAW_RP_BIAS_THR_GAIN` if roll/pitch yaw coupling becomes much worse at high throttle.
9. `YAW_ROLL_COUPLING_FF` / `YAW_PITCH_COUPLING_FF` only if one direction of roll or pitch yaws differently from the other direction.

Use small steps, for example `1.0f` to `3.0f`, and reverse the sign if the correction worsens the yaw. If hover still yaws clockwise with yaw stick centered, make `YAW_THROTTLE_HIGH_FF` slightly more negative first. If yaw appears mainly during roll/pitch, make `YAW_RP_OUTPUT_BIAS_FF` or `YAW_RP_RATE_BIAS_FF` more negative first.

If yaw correction appears to fight in the wrong direction during a no-prop bench test, change only `YAW_MIX_SIGN` from `1.0f` to `-1.0f` and retest. Do not fly until the no-prop yaw reaction is correct: when the frame is manually twisted clockwise with yaw stick centered, the controller must command the motor pair that produces counter-clockwise body torque.

Angle mode still uses expo for easy hover practice, but the current build restores practical roll/pitch authority so the frame is easier to place in real flight without feeling too soft. This does not provide GPS/optical-flow position hold; use small hover-trim changes for fixed drift and pilot stick corrections for wind or translational drift.

Hover roll trim is set slightly positive (`HOVER_ROLL_TRIM_DEG = 0.5`) to counter the observed drift toward the left `M2-M3` edge. Hover pitch trim is set positive (`HOVER_PITCH_TRIM_DEG = 1.6`) to counter the observed tendency to drift toward the rear `M3-M4` edge. Small takeoff-only roll/pitch trims are active while throttle is in the liftoff range and sticks are centered, then fade out above low hover throttle.

If hover still drifts toward `M2-M3`, increase `HOVER_ROLL_TRIM_DEG` or `TAKEOFF_ROLL_TRIM_DEG` in small `0.3 deg` steps. If it starts drifting toward `M1-M4`, reduce the roll trim or reverse the sign. If hover still drifts toward `M3-M4`, increase `HOVER_PITCH_TRIM_DEG` or `TAKEOFF_PITCH_TRIM_DEG` in small `0.3 deg` steps. If it starts drifting toward the front `M1-M2`, reduce the pitch trim or reverse the sign.

Throttle input is also smoothed and curved to make lift-off, altitude adjustments, and landing less abrupt around the hover range. A limited tilt-throttle boost now adds vertical-thrust compensation during roll/pitch, but this is not altitude hold.

This build intentionally removes barometer support from runtime control so failsafe and landing behavior depend only on attitude, throttle, RF state, and magnetometer-assisted heading hold. Heading hold is active when yaw stick returns to center and throttle is above the takeoff threshold; the compass is used only as a slow yaw-drift correction when healthy using the fixed offsets captured with `uav_magcal_tool`, with slightly stronger filtering and a modest correction gain for better long-term heading stability.

With only GY-87 installed, the firmware can stabilize attitude and heading, reduce drift from RC/sensor bias, and reduce altitude sag during tilt. It cannot hold absolute position or absolute altitude until GPS/optical-flow and MS5611/range sensing are added.
