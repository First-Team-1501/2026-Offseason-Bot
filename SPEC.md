# SPEC: FRC 1501 Offseason Bot

## 1. Scope

- Base: FRC 7457 suPURDUEper 2026 robot code (`suPURDUEper/2026-Robot-Code`, commit `20d4a19`).
- Target: FRC 1501 mechanical copy of the 7457 robot. Team number 1501.
- Hardware deltas vs. 7457: CAN map on two CANivores, Kraken X60 shooter flywheels (7457: X44), SDS MK5n R2
  swerve, one Limelight 4 (7457: three), driver input selectable: Xbox or 1501 joysticks (7457: Xbox), no CANdle,
  no climber.
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
| 1 | Shooter motor (follower, Opposed) | `SHOOTER_THREE` | `Shooter` |
| 2 | Shooter motor (follower, Opposed) | `SHOOTER_FOUR` | `Shooter` |
| 3 | Feeder motor (leader) | `FEEDER_ONE` | `Feeder` |
| 4 | Shooter motor (follower, Opposed) | `SHOOTER_FIVE` | `Shooter` |
| 5 | Indexer | `INDEXER` | `Hopper` |
| 6 | Intake motor (leader) | `INTAKE_ONE` | `Intake` |
| 7 | Intake motor | `INTAKE_TWO` | `Intake` |
| 8 | Main feeder roller | `UPTAKE` | `Uptake` |
| 9 | Hood motor | `SHOOTER_HOOD` | `ShooterHood` |
| 10 | Feeder motor | `FEEDER_TWO` | `Feeder` |
| 11 | Shooter motor (follower, Aligned) | `SHOOTER_TWO` | `Shooter` |
| 12 | Shooter motor (leader) | `SHOOTER_ONE` | `Shooter` |

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
- Heading: seeded from MegaTag1 while disabled (2+ tags within 4 m, 5 frames within 3 deg); `rezeroFieldHeading`
  (section 2.4) in teleop sets it with the intake pointing away from the driver station; Choreo autos reset it at
  start.
- Camera names: `Constants.VisionConstants.kLimelightNames`.

### 2.4 Driver controls

Hold-to-run, 7457 action semantics. `Constants.DriverConstants.kXboxDriver` selects one driver input, never both:
`true` (current) = Xbox controller on DS port 0 (`state/DriverXbox.java`); `false` = 1501 joysticks on DS ports 0
and 1 (`state/DriverSticks.java`). Both extend `DriverControls`, which applies the stick response; button numbers and
raw axes are defined only in these two classes.

| Action (`RobotStates`) | Xbox, port 0 | Joysticks, ports 0 and 1 | 7457 Xbox |
|---|---|---|---|
| translate | left stick | port 0 stick Y / X | left stick |
| rotate (overridden while aiming) | right stick X | port 1 stick X | right stick X |
| `actionIntake` | LT | port 0 btn 16 | LB |
| `actionAim` (aim + spin up) | A | port 1 btn 18 | RT |
| `actionShoot` (spin up + feed) | RT | port 0 btn 18 | RB |
| `actionPurge` | LB | port 0 btn 17 | LT |
| `actionSetShot` (fixed shot) | X (Y until 2026-10-22) | port 1 btn 17 | Y |
| `actionZeroHood` | D-pad down | port 1 btn 1 | D-pad down |
| `rezeroFieldHeading` | Back | port 0 btn 1 | Back |
| `actionDefenseToggle` (press: defense mode on/off, section 2.8) | Y | - | - |
| `actionHoodMax` (hold: hood at max angle, section 2.8) | RB | - | - |

- Xbox LT, A and RT: team (2026-10-21). Y defense toggle, RB hood-max hold, X fixed shot: team (2026-10-22). LB,
  D-pad down and Back: defaults pending team confirmation.
- Driver controller: GameSir in XInput mode; the DS lists it as "Controller (Xbox 360 for Windows Controller)", so
  the WPILib `XboxController` map applies (A btn 1, LB 5, Back 7; LT / RT axes 2 / 3; right X axis 4; D-pad POV 0).
- Xbox triggers count as pressed above `kXboxTriggerThreshold` (0.5, WPILib default).
- Stick response, both inputs, per axis (`DriverControls.shape`; 1501 2026 comp bot curve
  `DriveSticks.applyExponential`, there deadband 0.05 and exponent 3): 0 below deadband d, else
  sign(x) * ((|x| - d) / (1 - d))^n; then 7457 `kSlowModeScalor` 0.85. Tuned with the driver (2026-10-21): start
  d 0.02, n 3.0 (deadzone too large); now d = 0.01 (`kStickDeadband`, just above one DS axis count, 1/127) and
  n = 2.0 (`kStickExponent`, team). Tuning mode: live as
  `/Tuning/Driver/Stick Deadband` (clamped 0-0.3) and `/Tuning/Driver/Stick Exponent` (clamped 1-5); the driver tunes
  on the robot, then the final values go to `Constants`.
- Port 3 Xbox: test controller (`TestController`; 7457 `Driver`, port 2). Bound outside disabled: feeder A stop,
  B 500, X 1000, Y 2000 RPM; hood open-loop voltage on left stick Y. The DS assigns each USB device the first free
  port: check the DS USB tab order before enabling.
- Port 2 (1501 button board) unused.

### 2.5 Hood and flywheel interlock

Requirement (team, 2026-10-21): the shooter flywheel is never powered while the hood is fully down.

| Item | Definition |
|---|---|
| Fully down | `kReverseSoftLimit` (7 deg; bottom stop 6.837 deg) |
| Stow angle | `kReverseSoftLimit` + `kStowMarginDeg` (2.0 deg since 2026-10-21: at 1.5 deg the hood still hit occasionally while idling, team; live `ShooterHood/Stow Margin (Deg)` in tuning mode, clamped 0.5-5) |
| Hood down (`ShooterHood.isDown`) | measured angle below `kReverseSoftLimit` + half the stow margin |
| Interlock (`Shooter.outputInhibited`) | while the hood is down, every velocity/voltage request becomes `NeutralOut` (coast); `isAtVelocity` is false, so no feed. Running commands resume when the hood rises. Flywheel only; feeder, uptake and hopper are not interlocked. Since 2026-10-22 implemented through the `TalonFXSubsystem.outputInhibited` hook shared with defense mode (section 2.8). |
| Rest angle (`ShooterHood.restAngle`) | auto: fully down (7457 paths pass the trench with it down); teleop: fully down inside a trench zone, else stow |
| Trench zone (`FieldCalculations.inTrenchZone`) | robot center within trench depth / 2 + 1.0 m of either hub line in x, and within the trench opening width + 0.2 m of the side wall in y; all four trenches |
| Teleop in a trench zone | aim, shoot and fixed-shot hood targets are replaced by fully down, so the hood cannot rise under a trench; the interlock then blocks the flywheel |
| Zeroing (`actionZeroHood`) | drives to the stop (interlock active), re-zeroes, returns to the rest angle |
| Manual shot tuning | `Shot Tuning/Angle (Deg)` minimum is the stow angle |
| Telemetry | `Shooter/Hood Interlock`, `ShooterHood/Down`, `ShooterHood/In Trench Zone`, `ShooterHood/Rest Angle (Deg)` |

Consequence: in auto the flywheel is unpowered between volleys (hood down) and spins up again when the hood rises
for the next shot; `auto_rev` has no effect while the hood is down.

### 2.6 X-lock while shooting with auto-aim

Requirement (team, 2026-10-21): while shooting with auto-aim, the wheels X-lock so other robots cannot push the robot
off target. 7457's version (lock within 0.25 deg) was commented out.

| Item | Definition |
|---|---|
| Active | auto-aim request (`actionAim` or `auto_aim`) with shoot requested (`actionShoot` or `auto_shoot`) |
| Lock | aim error < `kAimLockEnterDeg` (1.0), driver translation command < `kAimLockMaxTranslationMps` (0.2 m/s), yaw rate < `kAimLockMaxTurnRateRadPerSec` (0.3) |
| Release | aim error > `kAimLockExitDeg` (3.0; heading controller re-aims, then locks again), driver translates, or shoot released |
| Request | `SwerveDriveBrake` (modules in X, drive output 0) |
| Telemetry | `AutoAim/X Lock` (DogLog), `/Tuning/Heading/Locked` |

### 2.7 Mechanisms by action

Requirement (team, 2026-10-21): while intaking only the intake runs (the shooter may idle); the feeder and the indexer
(and the uptake, the main feeder roller) run only while actively shooting.

| Action | Intake | Hopper (indexer) | Uptake | Feeder | Shooter |
|---|---|---|---|---|---|
| none | off | off | off | off | idle 700 RPM (unpowered with the hood down) |
| intake (`actionIntake`, `auto_intake`) | in, 4000 RPM target | off | off | off | idle |
| aim (`actionAim`) | off | off | off | off | shot speed |
| shoot (`actionShoot`, `auto_shoot`, `actionSetShot`) | in, 1700 RPM | runs once the shooter is at speed and in range (`auto_shoot`: at once) | shot speed once at speed and in range | shooter setpoint | shot speed |
| purge (`actionPurge`) | out, -8 V | off | off | off | idle |

### 2.8 Defense mode and hood-max hold

Requirement (team, 2026-10-22): a toggle that makes the robot a drive base ("shut off the top CAN bus"), and a hold
that raises the hood to its max angle, also in defense mode, and returns to normal operation on release.

| Item | Definition |
|---|---|
| Toggle | Xbox Y (`actionDefenseToggle`, teleop); state `RobotStates.defenseMode`, dashboard `Defense Mode`, log `States/Defense`. Cleared when auto starts. Not mapped on the joysticks. |
| Off in defense mode | Intake, Hopper, Feeder, Uptake, Shooter: every request becomes `NeutralOut` (`outputInhibited`, also applied each loop in `TalonFXSubsystem.periodic`). `actionIntake`, `actionAim`, `actionShoot`, `actionPurge`, `actionSetShot` are false, so there is no auto-aim heading control and no X-lock. |
| Unchanged in defense mode | Drive, hood (stow, fully down in trench zones, zeroing), Limelight. The upper CANivore stays powered and read (software cannot power it off; the hood is on it). |
| Hood-max hold | Xbox RB (`actionHoodMax`, teleop, also in defense mode): target `kForwardSoftLimit` (41.5 deg) replaces every teleop hood target while held (`ShooterHood.withMaxHold`); on release the running command's own target applies (aim angle while aiming, else rest angle). In a trench zone the hood stays fully down (team). |
| Shooting while held | not blocked: aim + shoot with RB held fires at the max angle. |

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
| 15 | Live RPM multipliers: SmartDashboard `Shooter/LUT RPM Scale` (default 1.21 since row 38; 1.10 since `3832fc4`; 1.20 before), `Shooter/Forward RPM Scale` (1.10 since row 38; 1.0 before) | `Shooter.java`, `Constants.java` |
| 16 | Vision: NT flush, MT2 gates, MegaTag1 heading seed while disabled, camera-pose check, telemetry; disabled throttle 50 | `Vision.java`, `Robot.java` |
| 17 | Intake runs inward at 1700 RPM while shooting (7457: full intake speed); sign corrected in row 34 | `Intake.java`, `Constants.java` |
| 18 | Build info logged 1 s into the main loop and on auto/teleop init (the data log is paused during the >10 s robot construction, so robotInit values never reached it) | `Robot.java` |
| 19 | Shot speed and hood angle from one place (`ShotCalculator`): hub and forwarding tables, dashboard manual shot tuning with point recording; `tuningMode` now only affects gains | `ShotCalculator.java` (new), `Shooter.java`, `ShooterHood.java` |
| 20 | Limelight robot-space pose sent from code (`VisionConstants.kCamera*`) at startup and on disable | `Constants.java`, `Vision.java`, `Robot.java` |
| 21 | Hood and flywheel interlock, stow angle, trench zone (section 2.5) | `Constants.java`, `ShooterHood.java`, `Shooter.java`, `FieldCalculations.java`, `ShotCalculator.java` |
| 22 | DogLog writes on the 250 Hz odometry thread decimated to 50 Hz (`DriveTelemetry`, `AutoAim`); they overflowed the DogLog queue (`MAX_QUEUED_LOGS` on `DriveState/ModuleStates`) | `DriveTelemetry.java`, `AutoAim.java` |
| 23 | `Constants.tuningMode` true for tuning sessions (set false for events); aim heading gains live as `/Tuning/AutoAim/Kp|Ki|Kd`, applied values logged as `AutoAim/Applied Gains` | `Constants.java`, `DriveStates.java`, `Drivetrain.java` |
| 24 | `Shot Tuning/Record` line also written to the DataLog `messages` entry (not subject to DogLog queue drops) | `ShotCalculator.java` |
| 25 | Tuning telemetry for all 21 TalonFX, power and heading, written directly to the DataLog under `/Tuning` (section 8) | `lib/telemetry/*` (new), `TalonFXSubsystem.java`, `VelocitySubsystem.java`, `PositionSubsystem.java`, `Shooter.java`, `ShooterHood.java`, `Drivetrain.java`, `DriveTelemetry.java`, `AutoAim.java`, `DriveStates.java`, `Robot.java` |
| 26 | [R] Followers created through `TalonFXSubsystem.addFollower` (telemetry registration, live limits); `Shooter` per-motor DogLog block removed (replaced by `/Tuning`) | `TalonFXSubsystem.java`, `Shooter.java` |
| 27 | [R] Live current limits for every mechanism and live gains plus limits for swerve drive/steer through one helper, `TunableTalonFXConfig` (`/Tuning/<name>/Stator Limit`, `Supply Limit`, `Supply Lower Limit`, `Supply Lower Time`; swerve also `Kp`..`Ka`) | `TunableTalonFXConfig.java` (new), `TalonFXSubsystem.java`, `Drivetrain.java` |
| 28 | CTRE SignalLogger (hoot) off: `enableAutoLogging(false)`, `stop()`; `SignalLogger.start()` and `write*` calls removed | `Robot.java`, `DriveTelemetry.java` |
| 29 | X-lock while shooting with auto-aim (section 2.6) | `AutoAim.java`, `DriveStates.java`, `Constants.java` |
| 30 | Startup re-application of unchanged tunable gains removed (defaults recorded at construction) | `VelocitySubsystem.java`, `PositionSubsystem.java` |
| 31 | Shooter `kV` 0.171 and idle 700 RPM from the 2026-10-21 log analysis (section 4); follower `Voltage` logged | `Constants.java`, `MotorTelemetry.java` |
| 32 | Loop-overrun reductions (about 90 overruns/min before; rio CPU saturated): CTRE template module Mechanism2d widgets removed (about 2000 NT writes/s from the odometry thread); NT DriveState and Field2d pose at 50 Hz, Field2d type set once (Limelight orientation stays at 250 Hz for MegaTag2); main thread at real-time priority for all of `robotPeriodic` (was the scheduler only); `HootAutoReplay` removed (hoot logging is off) | `DriveTelemetry.java`, `Robot.java` |
| 33 | [R] Driver input selected by `kXboxDriver` (section 2.4): Xbox on port 0 or 1501 joysticks on ports 0 and 1, behind abstract `DriverControls` (button map and raw axes per input; one stick curve, live deadband and exponent, now 0.01 and 2.0); 7457 `Driver` (now the test controller only) renamed `TestController`, so its port 3 disconnect alert no longer reads "Driver" | `DriverControls.java`, `DriverXbox.java` (new), `DriverSticks.java`, `TestController.java` (was `Driver.java`), `RobotStates.java`, `RobotContainer.java`, `DriveStates.java`, `Constants.java` |
| 34 | Intake direction: leader `inverted` false. With 7457's true, the intake action pushed balls out on the 1501 robot (team, 2026-10-21), while purge and the 2026-10-17 shooting value (-1700 RPM) pulled in. `kShootVelocity` +1700 keeps the shooting direction physically unchanged; purge now ejects. Intake velocity and voltage signs in logs before this build are opposite | `Intake.java`, `Constants.java` |
| 35 | [R] Only the intake runs while intaking (section 2.7): hopper agitation (+0.5 V) and uptake reverse (-6 V) on `actionIntake` removed. Feeder: periodic state machine (`FeedState`) replaced by a `feed()` command (shooter setpoint) held while shooting, NeutralOut at the end. 7457 fed on aim or shoot and then idled at 500 RPM indefinitely; its `stop` state built a command in `periodic` and discarded it, so the feeder kept its last request. Unused `FeederConstants` feed, purge and idle velocities removed | `Feeder.java`, `Hopper.java`, `Uptake.java`, `Constants.java` |
| 36 | From the `FRC_20261021_222140` analysis, first run after the repair sign-off (team approved 2026-10-21; both low confidence): Hopper supply limit 40 -> 36 A (limit reached 0.6% of enabled time, p99 36.7 A, targets pass with 30% margin); ShooterHood kP 800 -> 880 (settle p90 0.37 s against 0.30 s, overshoot p90 0.18 deg) | `Constants.java` |
| 37 | Stow margin 1.5 -> 2.0 deg (rest angle 9.0 deg, hood-down threshold 8.0 deg): the hood still hit occasionally while idling (team, 2026-10-21) | `Constants.java` |
| 38 | Shot speeds +10% (team, 2026-10-22: shots short around 3 m): `kLutRpmScale` 1.10 -> 1.21, `kForwardRpmScale` 1.0 -> 1.10, fixed set shot 1425 -> 1570 RPM (inline literal moved to `ShooterConstants.kSetShotRPM`); flywheel `kP` 0.25 -> 0.5 (section 4); hub hood angle at 2.5 m 17.5 -> 16.5 deg (team: angle a little too high on the 2.56 m shot; 2.56 m interpolates to 17.1 deg, was 17.9) | `Constants.java`, `Shooter.java` |
| 39 | Defense mode (Xbox Y toggle) and hood-max hold (Xbox RB); fixed shot moved Y -> X (section 2.8). Refactor: one `outputInhibited` hook in `TalonFXSubsystem`/`VelocitySubsystem` replaces the three `Shooter` request overrides of the hood interlock and carries defense mode for five subsystems | `TalonFXSubsystem.java`, `VelocitySubsystem.java`, `Shooter.java`, `Intake.java`, `Hopper.java`, `Feeder.java`, `Uptake.java`, `ShooterHood.java`, `RobotStates.java`, `DriverControls.java`, `DriverXbox.java`, `DriverSticks.java`, `RobotContainer.java` |
| 40 | Hub RPM table 1.75 / 2.0 / 2.25 m: 1340 / 1365 / 1390 -> 1300 / 1325 / 1370 (team, 2026-10-22: with x1.21 the 2.01-2.16 m volley in `FRC_20261022_221806` was a little long; 2.54 m and 2.88-3.02 m good). Effective at 2.0 m 1652 -> 1603 RPM. Analyzer proposal Feeder supply limit 40 -> 36 A (low) declined by the team | `Constants.java` |

Notes:

- #3: 7457 defined drivebase IDs twice (`CanId`, unused; `TunerConstants`), with BL/BR CANcoder IDs swapped
  between them. Regenerating `TunerConstants` with Tuner X requires re-applying the `CanId` references.
- #5: config cleanup. Duty-cycle limits do not apply to the torque-current or voltage requests these
  subsystems use; no runtime change.
- #6: replaces per-camera triplicated code; adding a camera is one list entry.
- #8: 7457 constructed `Uptake` twice, creating two subsystems commanding TalonFX `UPTAKE`.

## 4. Kraken X60 flywheel gains

- Control: `VelocityVoltage`, FOC off (7457: `VelocityTorqueCurrentFOC` bang-bang, `kP` 99999). Gains in V
  per mechanism rot/s: `kP` 0.5 (0.25 until 2026-10-22), `kS` 0.2, `kV` 0.171.
- 2026-10-22 `kP` 0.25 -> 0.5 (team approved): in the `FRC_20261022_220137` volley (2.56 m, 1563 RPM, 10 balls) the
  flywheel dipped 7-14% per ball, 90% recovery p90 0.42 s, while the loop applied only 4.6-5.0 V (stator peak 47 A of
  50 A; battery 7.2-10 V). Closed-loop time constant with `kA` 0.0214: about 32 ms. Verify dips, recovery, stator-limit
  time and battery minimum in the next volley log.
- 2026-10-21 refit (first post-sign-off log `FRC_20261021_211040`, 913-1622 RPM, two independent fits, R^2 >= 0.995):
  `kS` 0.198-0.212 (unchanged), `kV` 0.1705-0.1713 (was 0.18; the flywheel ran 18-37 RPM fast). `kP` unchanged:
  per-ball dips of 55-203 RPM recovered to within 25 RPM in 0.02-0.10 s, stator limit reached in under 2% of feeding.
- Idle `kIdleRPM` 700 (was 900): idle drew about 12 A supply at 900 RPM; spin-up from 900 took 0.22-0.32 s of the
  1.0 s target. Pending verification in the next log.
- Load share (2026-10-21): leader 12 carries about 3x the stator and supply current of each follower at the same speed
  (18.8 A vs 6.2-6.6 A at 900 RPM); mechanical check pending; follower voltage now logged.
- `kS` 0.55 was fitted from the 2026-10-17 logs (900-1725 RPM; with `kS` 0.2 the flywheel settled 40-70 RPM below
  target) while follower motor 1 braked the flywheel at the current limit. After the direction fix (`3832fc4`) `kS`
  returned to 0.2 until a refit from new logs; fitted `kV` 0.180 equals the existing value.
- Feeding waits for `Shooter/At Velocity`: at least 1000 RPM and within 100 RPM of target.
- Unchanged: `kMaxAmps` 50 A stator limit, supply limit 40 A, ratio 36:24, distance LUTs (7457 values).
- X60 free speed approx. 6000 RPM -> flywheel limit approx. 4000 RPM at 1.5:1; LUT maximum 2075 RPM
  (2511 RPM with the 1.21 multiplier).

## 5. Bring-up checklist

1. Tuner X: re-ID upper devices per section 2.2 (device names already equal target IDs).
2. Tuner X: CANivore names exactly `Upper Bot CANivore` and `Lower Bot Canivore`. A "CAN bus not found"
   DS error indicates a case or spacing mismatch.
3. Tuner X: Phoenix Pro license active for all upper-bus TalonFX (CANivore or per-device). Unlicensed
   devices produce no output under torque-current requests (fault `UnlicensedFeatureInUse`).
4. Swerve: measure CANcoder offsets (Tuner X swerve generator or manual zero); update `k*EncoderOffset` in
   `TunerConstants.java`. Verify inversions, gear ratios (MK5n R2: drive 6.0268, steer 26.09, couple 3.857),
   wheel radius 2 in, module positions. Front = intake end: translate forward after `rezeroFieldHeading` and the
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
8. Hood: zero with `actionZeroHood` (section 2.4); soft limits 7 deg to 41.5 deg (0.115234 mechanism rot, measured top of travel
   on the 1501 hood). Hood angles depend on the hood resting on its bottom stop when robot code starts (or after
   zeroing). Interlock check (section 2.5): teleop enable -> hood at stow, flywheel may idle; zeroing ->
   `Shooter/Hood Interlock` true and flywheel unpowered, then stow; trench zone -> hood fully down, flywheel
   unpowered, aim/shoot hold the hood down; auto -> flywheel unpowered whenever the hood is down. Set the stow margin
   live until the team confirms clearance, then copy it to `kStowMarginDeg`.
9. Intake, hopper, feeder, uptake: verify direction and follower flags (7457 values retained). Intake corrected
   2026-10-21 (section 3 row 34; team confirmed). Hopper and uptake no longer run while intaking (row 35).
10. LL4: hostname `limelight`, static IP 10.15.1.11; robot-space camera pose (section 2.3, measured from the
    center of the module rectangle); AprilTag pipeline 0 with Full 3D; field map matching the event field
    (`FieldConstants.fieldType`); compare `Vision/limelight Pose (mt2)` and `Distance To Goal` against
    tape-measured positions, standing still and rotating in place.
11. DS USB tab: `kXboxDriver` true: Xbox in slot 0; false: drive stick in slot 0, rotation stick in slot 1. Verify
    buttons per section 2.4.

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
- Alerts: an unplugged test Xbox (port 3) raises "TestController Gamepad Disconnected" (7457: "Driver Gamepad
  Disconnected", port 2). The driver input (Xbox or joysticks) has no disconnect alert.
- Distance LUTs are 7457's, scaled by `Shooter/LUT RPM Scale`; the forwarding LUT is an untuned estimate.
- Choreo `deploy/choreo/Tests.chor` module positions (x 12.375 in, y 9.375 in) are swapped relative to
  `TunerConstants` (x 9.375 in, y 12.375 in); robot mass 143.8 lb (7457). Verify against the 1501 robot
  before regenerating trajectories.
- `FieldConstants.fieldType` = `WELDED`; set per event field.
- Phoenix 6 vendordep 26.3.0; TalonFX firmware 26.3.0.0.
- `Climber` retained, not constructed; climber triggers bound to `Gamepad.kFalse`.

## 8. Tuning telemetry and continuous tuning

- Purpose: every motor's gains and current limits are tuned continuously from logs (laptop tools in `tools/tuning`;
  system spec in the vault: `Offseason Bot/Continuous Tuning System`). Gain and limit changes need team approval and
  use only data recorded after the team's mechanical sign-off, from mechanisms working as intended.
- Writer: `lib/telemetry` (`TuningTelemetry.periodic()` from `Robot.robotPeriodic`), straight to the WPILib DataLog
  (no DogLog queue, no NT); values rounded and written only on change.
- Per motor `/Tuning/<Mechanism>/<CAN id>/`: leaders `Voltage`, `Velocity`, `Reference`, `StatorCurrent`,
  `SupplyCurrent`, `ControlMode`, `Temp` (1 Hz); position leaders add `Position`; followers `Voltage`, `Velocity`,
  `StatorCurrent`, `SupplyCurrent`, `Temp`; swerve motors `Voltage`, `StatorCurrent`, `SupplyCurrent`, `ControlMode`,
  `Temp`. Mechanism signals requested at 100 Hz, logged at 50 Hz while enabled and 2 Hz while disabled; `ControlMode`
  and `MechanismOK` at 10 Hz.
- Per mechanism: `Config` (JSON: gains, ratio, current limits, inversion, neutral mode, tolerance, extras) at 1 Hz
  and on change; `MechanismOK` from dashboard `Tuning/Mechanism OK/<Mechanism>` (default true; set false while a
  mechanism is known faulty; resets to true on code restart).
- Swerve arrays (FL, FR, BL, BR) at 125 Hz on change: `/Tuning/SwerveSteer/Angles`, `Targets` (deg),
  `/Tuning/SwerveDrive/Speeds`, `TargetSpeeds` (m/s).
- Heading at 250 Hz while auto-aim runs: `/Tuning/Heading/Target`, `Measured`, `Error` (deg, signed), `Output`
  (rad/s), `Locked`; `Active` (50 Hz), `Config` (1 Hz).
- Power at 50 Hz: `/Tuning/Power/BatteryVoltage`, `BrownedOut`, `Enabled`, `Mode`, `PDH/TotalCurrent`, `PDH/Voltage`,
  `PDH/Currents` (PDH polling stops if no module answers).
- Cost: `/Tuning/Telemetry Time (ms)` (simulation: median 0.08 ms).
