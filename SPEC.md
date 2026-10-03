# SPEC: FRC 1501 Offseason Bot

## 1. Scope

- Base: FRC 7457 suPURDUEper 2026 robot code (`suPURDUEper/2026-Robot-Code`, commit `20d4a19`).
- Target: FRC 1501 mechanical copy of the 7457 robot. Team number 1501.
- Hardware deltas vs. 7457: CAN map on two CANivores, Kraken X60 shooter flywheels (7457: X44), one
  Limelight 4 (7457: three), 1501 driver joysticks (7457: Xbox), no CANdle, no climber.
- Toolchain: WPILib 2026.2.1, Phoenix 6 26.1.1 (Pro licensed), Java 17.

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

- One Limelight 4, hostname `limelight`, facing the shooter (robot rear). Auto-aim (`AutoAim.java`) points
  the robot rear at the target; 7457's rear camera `limelight-b` was its only pose source.
- MegaTag2 with external IMU: `SetIMUMode(name, 0)`; heading published to `robot_orientation_set` by
  `DriveTelemetry`.
- Camera names: `Constants.VisionConstants.kLimelightNames`.

### 2.4 Driver controls

Hold-to-run, 7457 action semantics. Button numbers defined only in `state/DriverSticks.java`.

| Input | Action (`RobotStates`) | 7457 Xbox equivalent |
|---|---|---|
| Port 0 drive stick Y / X | translate | left stick |
| Port 0 btn 1 | `rezeroFieldHeading` | Back |
| Port 0 btn 16 | `actionSetShot` (fixed shot) | Y |
| Port 0 btn 17 | `actionPurge` | LT |
| Port 0 btn 18 | `actionAim` (aim + spin up) | RT |
| Port 1 rotation stick X | rotate (overridden while aiming) | right stick X |
| Port 1 btn 1 | `actionZeroHood` | D-pad down |
| Port 1 btn 17 | `actionIntake` | LB |
| Port 1 btn 18 | `actionShoot` (feed) | RB |
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

Notes:

- #3: 7457 defined drivebase IDs twice (`CanId`, unused; `TunerConstants`), with BL/BR CANcoder IDs swapped
  between them. Regenerating `TunerConstants` with Tuner X requires re-applying the `CanId` references.
- #5: config cleanup. Duty-cycle limits do not apply to the torque-current or voltage requests these
  subsystems use; no runtime change.
- #6: replaces per-camera triplicated code; adding a camera is one list entry.
- #8: 7457 constructed `Uptake` twice, creating two subsystems commanding TalonFX `UPTAKE`.

## 4. Kraken X60 flywheel gains

- Control: `VelocityTorqueCurrentFOC` (Phoenix Pro). `kP` 99999 saturates output to
  `PeakForwardTorqueCurrent` (50 A) below setpoint and to `PeakReverseTorqueCurrent` (0 A) above it;
  `kS` / `kV` set steady-state current only.
- TalonFX auto-detects the integrated motor; `TalonFXConfiguration` has no motor-type setting.
- Torque constant Kt (WPILib `DCMotor` 2026.2.1, FOC): X44 5.01 N*m / 329 A = 0.01523 N*m/A;
  X60 9.37 N*m / 483 A = 0.01940 N*m/A. Ratio 0.785.
- `kS` 9.0 -> 7.06 A; `kV` 0.15 -> 0.118 A/(rot/s). Starting values; tune on robot (section 5).
- Unchanged: `kP`, `kMaxAmps` 50 A (stator limit and peak torque current), supply limit 40 A, ratio 36:24,
  velocity tolerance 100 RPM, distance LUTs.
- At 50 A the X60 produces 0.97 N*m per motor vs. 0.76 N*m for the X44 (+27%): faster spin-up and
  recovery than 7457.
- X60 FOC free speed 5800 RPM -> flywheel limit approx. 3870 RPM at 1.5:1; LUT maximum 2075 RPM.

## 5. Bring-up checklist

1. Tuner X: re-ID upper devices per section 2.2 (device names already equal target IDs).
2. Tuner X: CANivore names exactly `Upper Bot CANivore` and `Lower Bot Canivore`. A "CAN bus not found"
   DS error indicates a case or spacing mismatch.
3. Tuner X: Phoenix Pro license active for all upper-bus TalonFX (CANivore or per-device). Unlicensed
   devices produce no output under torque-current requests (fault `UnlicensedFeatureInUse`).
4. Swerve: measure CANcoder offsets (Tuner X swerve generator or manual zero); update `k*EncoderOffset` in
   `TunerConstants.java`. Verify inversions, gear ratios (drive 7.03125, steer 26.09), wheel radius 2 in,
   module positions.
5. Shooter direction and follower alignment (robot disabled):
   - Tuner X shows identical `MotorOutput.Inverted` on all five shooter TalonFX.
   - Spin flywheel by hand in the shooting direction; `Shooter/Current RPM` must read positive, else invert
     `Shooter.inverted()`.
   - Follower velocity sign equal to leader: `false` (Aligned); opposite: `true` (Opposed) in the
     `createPermanentFollowerTalon` calls in `Shooter.java`.
6. Shooter closed loop: set `Constants.tuningMode = true`; set `Shot Tuning/Speed (RPM)` to approx. 800;
   hold aim; verify `Shooter/At Velocity` and balanced stator current across the five motors; tune
   `Shooter/Ks`, `Shooter/Kv`; copy results to `ShooterConstants`; restore `tuningMode = false`.
7. Validate distance LUTs (`Constants.LookupTables`) with test shots; retune points as required.
8. Hood: zero with port 1 btn 1; verify soft limits (7 to 30 deg).
9. Intake, hopper, feeder, uptake: verify direction and follower flags (7457 values retained).
10. LL4: hostname `limelight`; robot-space camera pose (rear-facing, measured); AprilTag pipeline with
    MegaTag2; field map matching the event field (`FieldConstants.fieldType`); compare
    `Vision/limelight Pose (mt2)` against a known robot position.
11. DS: drive stick in USB slot 0, rotation stick in slot 1; verify buttons per section 2.4.

## 6. Known issues and limitations

- Simulation: Phoenix sim keys devices by type and ID only (`PlatformJNI.JNI_SimCreate(type, id)`).
  Upper and lower TalonFX IDs overlap (1, 2, 4, 5, 7, 8, 10, 11), so simulation is valid for startup and NT
  checks only.
- Alerts: an unplugged test Xbox (port 3) raises "Driver Gamepad Disconnected" (pre-existing in 7457 on
  port 2). The sticks have no disconnect alert.
- `TunerConstants.java` encoder offsets are 7457 values until measured (section 5, item 4).
- Choreo `deploy/choreo/Tests.chor` module positions (x 12.375 in, y 9.375 in) are swapped relative to
  `TunerConstants` (x 9.375 in, y 12.375 in); robot mass 143.8 lb (7457). Verify against the 1501 robot
  before regenerating trajectories.
- `FieldConstants.fieldType` = `WELDED`; set per event field.
- Phoenix 6 vendordep 26.1.1; TalonFX firmware 26.3.0.0. Vendordep update optional.
- `Climber` retained, not constructed; climber triggers bound to `Gamepad.kFalse`.
