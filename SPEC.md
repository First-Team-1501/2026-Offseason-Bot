# SPEC: FRC 1501 Offseason Bot

## 1. Scope

- Base: FRC 7457 suPURDUEper 2026 robot code (`suPURDUEper/2026-Robot-Code`, commit `20d4a19`).
- Target: FRC 1501 mechanical copy of the 7457 robot. Team number 1501.
- Hardware deltas vs. 7457: CAN map on two CANivores, Kraken X60 shooter flywheels (7457: X44), SDS MK5n R2
  swerve, one Limelight 4 (7457: three), 1501 driver joysticks (7457: Xbox), no CANdle, no climber.
- Toolchain: WPILib 2026.2.1, Phoenix 6 26.3.0 (mechanisms do not need Phoenix Pro), Java 17.

## 2. Hardware map

### 2.1 CAN buses

| Bus name (case-sensitive) | Devices | `Constants` field |
|---|---|---|
| `Upper Bot CANivore` | mechanisms | `upperBus` |
| `Lower Bot Canivore` | drivebase | `lowerBus` |

### 2.2 CAN IDs

Source of truth: `CanId.java`. `TunerConstants.java` reads drivebase IDs from `CanId`.

Upper Bot CANivore:

| ID | Device | `CanId` | Subsystem |
|---|---|---|---|
| 1 | Shooter motor (leader) | `SHOOTER_ONE` | `Shooter` |
| 2 | Shooter motor | `SHOOTER_TWO` | `Shooter` |
| 3 | Feeder motor (leader) | `FEEDER_ONE` | `Feeder` |
| 4 | Shooter motor | `SHOOTER_THREE` | `Shooter` |
| 5 | Indexer | `INDEXER` | `Hopper` |
| 6 | Intake motor (leader) | `INTAKE_ONE` | `Intake` |
| 7 | Intake motor | `INTAKE_TWO` | `Intake` |
| 8 | Main feeder roller | `UPTAKE` | `Uptake` |
| 9 | Hood motor | `SHOOTER_HOOD` | `ShooterHood` |
| 10 | Feeder motor | `FEEDER_TWO` | `Feeder` |
| 11 | Shooter motor | `SHOOTER_FOUR` | `Shooter` |
| 12 | Shooter motor | `SHOOTER_FIVE` | `Shooter` |

Lower Bot Canivore:

| ID | Device | `CanId` |
|---|---|---|
| 1 / 2 / 3 | Front left drive / steer / CANcoder | `TALONFX_DRIVE_FL` / `TALONFX_STEER_FL` / `CANCODER_STEER_FL` |
| 4 / 5 / 6 | Back left drive / steer / CANcoder | `TALONFX_DRIVE_BL` / `TALONFX_STEER_BL` / `CANCODER_STEER_BL` |
| 7 / 8 / 9 | Front right drive / steer / CANcoder | `TALONFX_DRIVE_FR` / `TALONFX_STEER_FR` / `CANCODER_STEER_FR` |
| 10 / 11 / 12 | Back right drive / steer / CANcoder | `TALONFX_DRIVE_BR` / `TALONFX_STEER_BR` / `CANCODER_STEER_BR` |
| 15 | Pigeon 2 | `PIGEON` |

Not present on the 1501 robot: `CANDLE` (29), `CLIMBER` (21). Entries retained because the unconstructed
`Lights` and `Climber` classes reference them.

### 2.3 Vision

- Robot front (+X) = intake end. One Limelight 4, hostname `limelight`, facing the shooter (robot rear).
  Auto-aim (`AutoAim.java`) points the robot rear at `FieldCalculations.aimPoint` (field-frame heading,
  both alliances); 7457's rear camera `limelight-b` was its only pose source.
- Limelight robot-space pose: LL Yaw about 180, LL Forward negative (camera behind center), LL Right positive to
  the robot's right. `Vision` ignores the camera and raises an alert if the pose is not rear-facing.
- MegaTag2 with external IMU: `SetIMUMode(name, 0)`; heading published to `robot_orientation_set` by
  `DriveTelemetry` and flushed each loop by `Vision`. MT2 frames are rejected above 90 deg/s, when their yaw
  differs from the pose at the frame time by more than 4 deg, off-field, or repeated.
- Heading: seeded from MegaTag1 while disabled (2+ tags within 4 m, 5 frames within 3 deg); port 0 btn 1 in
  teleop sets it with the intake pointing away from the driver station; Choreo autos reset it at start.
- Camera names: `Constants.VisionConstants.kLimelightNames`.

### 2.4 Driver controls

Hold-to-run, 7457 action semantics. Button numbers defined only in `state/DriverSticks.java`.

| Input | Action (`RobotStates`) | 7457 Xbox equivalent |
|---|---|---|
| Port 0 drive stick Y / X | translate | left stick |
| Port 0 btn 1 | `rezeroFieldHeading` | Back |
| Port 0 btn 16 | `actionIntake` | LB |
| Port 0 btn 17 | `actionPurge` | LT |
| Port 0 btn 18 | `actionShoot` (spin up + feed) | RB |
| Port 1 rotation stick X | rotate (overridden while aiming) | right stick X |
| Port 1 btn 1 | `actionZeroHood` | D-pad down |
| Port 1 btn 17 | `actionSetShot` (fixed shot) | Y |
| Port 1 btn 18 | `actionAim` (aim + spin up) | RT |
| Port 3 Xbox | test controller (`testController`) | port 2 |

- Stick response from 1501 2026 code: deadband 0.05, cubic, then 7457 `kSlowModeScalor` 0.85.
- Port 2 (1501 button board) unused.

## 3. Software changes vs. 7457

[R] = refactor for maintainability.

| # | Change | Files |
|---|---|---|
| 1 | Team number 1501 | `.wpilib/wpilib_preferences.json` |
| 2 | Two CANivore buses; 1501 CAN IDs; unused `CANCODER_HOOD` removed | `Constants.java`, `CanId.java` |
| 3 | [R] `TunerConstants` reads IDs and bus name from `CanId` / `Constants` | `TunerConstants.java` |
| 4 | X60 shooter feedforward gains (section 4) | `Constants.java` |
| 5 | Second `PeakForwardDutyCycle` assignment corrected to `PeakReverseDutyCycle` | `Shooter.java`, `Feeder.java` |
| 6 | [R] Limelight name list; single LL4 | `Constants.java`, `Vision.java`, `DriveTelemetry.java` |
| 7 | 1501 joystick input; test controller port 3 | `DriverSticks.java` (new), `RobotStates.java`, `RobotContainer.java`, `DriveStates.java`, `ShooterHood.java`, `Constants.java` |
| 8 | Duplicate `Uptake` construction removed | `RobotContainer.java` |
| 9 | `Lights` not constructed (no CANdle) | `RobotContainer.java` |
| 10 | Mechanisms use `VelocityVoltage` / `VoltageOut` with FOC off (no Phoenix Pro needed); gains in V per mechanism rot/s (section 4) | `VelocitySubsystem.java`, `TalonFXSubsystem.java`, `PositionSubsystem.java`, `Constants.java` |
| 11 | SDS MK5n R2 drive constants from the 1501 Tuner X project: drive 6.0268, couple 3.857, 5.12 m/s at 12 V | `TunerConstants.java` |
| 12 | Front = intake end; offsets measured in Tuner X with the shooter end as front, each kept with its module | `CanId.java`, `TunerConstants.java` |
| 13 | Auto-aim heading in field frame (`TargetDirectionPerspective.BlueAlliance`); `AutoAim/Error` correct on red | `AutoAim.java` |
| 14 | Forwarding: outside our zone, aim at a point in our zone and use `distanceToForwardRPM` / `Angle` (untuned) and uptake 2000 RPM | `FieldCalculations.java`, `AutoAim.java`, `Shooter.java`, `ShooterHood.java`, `Uptake.java`, `Constants.java` |
| 15 | Live RPM multipliers: SmartDashboard `Shooter/LUT RPM Scale` (default 1.20), `Shooter/Forward RPM Scale` (1.0) | `Shooter.java`, `Constants.java` |
| 16 | Vision: NT flush, MT2 gates, MegaTag1 heading seed while disabled, camera-pose check, telemetry; disabled throttle 50 | `Vision.java`, `Robot.java` |
| 17 | Intake reverses (-1700 RPM) while shooting | `Intake.java`, `Constants.java` |
| 18 | Build info logged 1 s into the main loop and on auto/teleop init (the data log is paused during the >10 s robot construction, so robotInit values never reached it) | `Robot.java` |
| 19 | Shot speed and hood angle from one place (`ShotCalculator`): hub and forwarding tables, dashboard manual shot tuning with point recording; `tuningMode` now only affects gains | `ShotCalculator.java` (new), `Shooter.java`, `ShooterHood.java` |
| 20 | Limelight robot-space pose sent from code (`VisionConstants.kCamera*`) at startup and on disable | `Constants.java`, `Vision.java`, `Robot.java` |

Notes:

- #3: 7457 defined drivebase IDs twice (`CanId`, unused; `TunerConstants`), with BL/BR CANcoder IDs swapped
  between them. Regenerating `TunerConstants` with Tuner X requires re-applying the `CanId` references.
- #5: config cleanup. Duty-cycle limits do not apply to the torque-current or voltage requests these
  subsystems use; no runtime change.
- #6: replaces per-camera triplicated code; adding a camera is one list entry.
- #8: 7457 constructed `Uptake` twice, creating two subsystems commanding TalonFX `UPTAKE`.

## 4. Kraken X60 flywheel gains

- Control: `VelocityVoltage`, FOC off (7457: `VelocityTorqueCurrentFOC` bang-bang, `kP` 99999). Gains in V
  per mechanism rot/s: `kP` 0.25, `kS` 0.55, `kV` 0.18.
- `kS` / `kV` fitted from steady-state flywheel speeds in the 2026-10-17 logs (900-1725 RPM): with `kS` 0.2 the
  flywheel settled 40-70 RPM below target at every speed; fitted `kV` 0.180 equals the existing value.
- Feeding waits for `Shooter/At Velocity`: at least 1000 RPM and within 100 RPM of target.
- Unchanged: `kMaxAmps` 50 A stator limit, supply limit 40 A, ratio 36:24, distance LUTs (7457 values).
- X60 free speed approx. 6000 RPM -> flywheel limit approx. 4000 RPM at 1.5:1; LUT maximum 2075 RPM
  (2490 RPM with the 1.20 multiplier).

## 5. Bring-up checklist

1. Tuner X: re-ID upper devices per section 2.2 (device names already equal target IDs).
2. Tuner X: CANivore names exactly `Upper Bot CANivore` and `Lower Bot Canivore`. A "CAN bus not found"
   DS error indicates a case or spacing mismatch.
3. Tuner X: Phoenix Pro license active for all upper-bus TalonFX (CANivore or per-device). Unlicensed
   devices produce no output under torque-current requests (fault `UnlicensedFeatureInUse`).
4. Swerve: measure CANcoder offsets (Tuner X swerve generator or manual zero); update `k*EncoderOffset` in
   `TunerConstants.java`. Verify inversions, gear ratios (MK5n R2: drive 6.0268, steer 26.09, couple 3.857),
   wheel radius 2 in, module positions. Front = intake end: push the drive stick forward after btn 1 and the
   robot must move intake-first.
5. Shooter direction and follower alignment (robot disabled):
   - Tuner X shows identical `MotorOutput.Inverted` on all five shooter TalonFX.
   - Spin flywheel by hand in the shooting direction; `Shooter/Current RPM` must read positive, else invert
     `Shooter.inverted()`.
   - Follower velocity sign equal to leader: `false` (Aligned); opposite: `true` (Opposed) in the
     `createPermanentFollowerTalon` calls in `Shooter.java`.
6. Shooter closed loop: with `Shot Tuning/Manual` on, set `Shot Tuning/Speed (RPM)` to 1300, 1700 and 2100
   (`At Velocity` is never true below 1000 RPM); hold aim; verify `Shooter/At Velocity` and balanced stator
   current across the five motors. Gains are fitted from the logs (section 4), or tuned live with
   `Constants.tuningMode = true` (`Tuning/Shooter/Ks`, `Kv`), copied to `ShooterConstants`, then
   `tuningMode = false`.
7. Validate distance LUTs (`Constants.LookupTables`) with test shots; retune points as required. Dashboard
   `Shot Tuning/Manual` on: the shot uses `Shot Tuning/Speed (RPM)` / `Shot Tuning/Angle (Deg)` (initialized from
   the table at the current distance; feeding needs at least 1000 RPM). When a shot scores, press
   `Shot Tuning/Record`: the log gets `Shot Tuning/Recorded Point` = [distance m, RPM, deg, forwarding] and the
   console a matching line. Distance is `Shot Tuning/Distance (m)` (robot center to hub center).
8. Hood: zero with port 1 btn 1; verify soft limits (7 to 30 deg).
9. Intake, hopper, feeder, uptake: verify direction and follower flags (7457 values retained).
10. LL4: hostname `limelight`, static IP 10.15.1.11; robot-space camera pose (section 2.3, measured from the
    center of the module rectangle); AprilTag pipeline 0 with Full 3D; field map matching the event field
    (`FieldConstants.fieldType`); compare `Vision/limelight Pose (mt2)` and `Distance To Goal` against
    tape-measured positions, standing still and rotating in place.
11. DS: drive stick in USB slot 0, rotation stick in slot 1; verify buttons per section 2.4.

## 6. Verification status

| Check | Result |
|---|---|
| `gradlew build` | Pass. One pre-existing warning: `Robot.java:139` deprecated `Command.schedule()`. |
| `gradlew simulateJava` startup | Pass: "Robot program startup complete", no exceptions. One startup burst of "CAN message is stale" for lower-bus devices (sim timing); none after init. |
| Limelight NT topics | Code inspection: all Limelight NT access derives from `kLimelightNames`. No runtime NT client used. |
| On-robot bring-up | Partial (2026-10-17, shooter-front frame): manual driving and shooting, MegaTag2 pose, hub auto-aim on red. After the switch to intake-front: re-enter the LL pose, then re-verify items 4, 10 and aim. Hub LUT and forwarding LUT not yet validated. |

## 7. Known issues and limitations

- Simulation: Phoenix sim keys devices by type and ID only (`PlatformJNI.JNI_SimCreate(type, id)`).
  Upper and lower TalonFX IDs overlap (1, 2, 4, 5, 7, 8, 10, 11), so simulation is valid for startup and NT
  checks only.
- Alerts: an unplugged test Xbox (port 3) raises "Driver Gamepad Disconnected" (pre-existing in 7457 on
  port 2). The sticks have no disconnect alert.
- Distance LUTs are 7457's, scaled by `Shooter/LUT RPM Scale`; the forwarding LUT is an untuned estimate.
- Choreo `deploy/choreo/Tests.chor` module positions (x 12.375 in, y 9.375 in) are swapped relative to
  `TunerConstants` (x 9.375 in, y 12.375 in); robot mass 143.8 lb (7457). Verify against the 1501 robot
  before regenerating trajectories.
- `FieldConstants.fieldType` = `WELDED`; set per event field.
- Phoenix 6 vendordep 26.3.0; TalonFX firmware 26.3.0.0.
- `Climber` retained, not constructed; climber triggers bound to `Gamepad.kFalse`.
