# Robot-Base

A minimal starting point for a new FTC robot project, built on [AlonLib](https://github.com/alonHamb/AlonLib)
and SolversLib's command-based framework. Clone this at the start of a season instead of starting
from a blank `FtcRobotController` checkout.

## What's here

- `FtcRobotController/` — the stock FTC SDK robot controller app (unmodified).
- `TeamCode/` — where you write robot code:
  - [`RobotMap.kt`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotMap.kt) — hardware
    device names/types, one nested object per subsystem.
  - [`RobotContainer.kt`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotContainer.kt) —
    wires up subsystems, default commands, and gamepad bindings.
  - `subsystems/example/` — a template subsystem wrapping one motor (`ExampleSubsystem.kt` +
    `ExampleConstants.kt`).
  - `commands/ExampleCommands.kt` — a template command-factory file (extension functions on the
    example subsystem).
  - `opmodes/teleop/BlueMainTeleop.kt` and `opmodes/teleop/RedMainTeleop.kt` — identical except
    each constructs its `RobotContainer` with the matching `Alliance` (`Blue`/`Red`).
  - `opmodes/autonomous/MainAutonomous.kt` — constructs a `RobotContainer` and schedules an example
    RoadRunner trajectory to prove the drivetrain works end to end.
  - `subsystems/drive/` — `DriveSubsystem` wraps RoadRunner's Mecanum drive (see below) for
    field/robot-centric teleop driving and `actionBuilder()` for autonomous trajectories.
  - `roadrunner/` — a Kotlin port of the official RoadRunner FTC quickstart, simplified to
    Mecanum+Pinpoint only (see below).

## RoadRunner

`roadrunner/MecanumDrive.kt` handles kinematics, pose tracking (via a goBILDA Pinpoint —
`roadrunner/PinpointLocalizer.kt`), and PID/feedforward trajectory following. Before trajectories
will track accurately you need to tune `MecanumDrive.PARAMS` (and `PinpointLocalizer.PARAMS` for
the pod offsets) for your robot:

1. Deploy the code and open the Driver Station's OpMode list — you'll see the `quickstart` group
   with `MecanumMotorDirectionDebugger`, `ForwardPushTest`/`LateralPushTest`,
   `ForwardRampLogger`/`LateralRampLogger`/`AngularRampLogger`, `ManualFeedforwardTuner`, and
   `ManualFeedbackTuner`, registered by `roadrunner/tuning/TuningOpModes.kt`.
2. Follow the tuning steps at [rr.brott.dev/docs/v1-0/tuning](https://rr.brott.dev/docs/v1-0/tuning/)
   in that order, editing the corresponding `PARAMS` fields (visible/editable live in FTC
   Dashboard) as you go.
3. `roadrunner/tuning/SplineTest.kt` and `LocalizationTest.kt` are useful smoke tests once tuned.

To schedule a trajectory from a command-based OpMode, build it from `DriveSubsystem.actionBuilder()`
and convert it to a `Command` with AlonLib's `Action.asCommand()` (see `MainAutonomous.kt` for an
example) instead of `Actions.runBlocking` (which is for plain `LinearOpMode`s only).

## Adding a real mechanism

1. Copy `subsystems/example/` to a new package named after the mechanism (e.g. `subsystems/arm/`),
   and rename `ExampleSubsystem`/`ExampleConstants` to match.
2. Add the mechanism's hardware IDs to `RobotMap.kt` as a new nested object.
3. Copy `commands/ExampleCommands.kt` to a new `<Mechanism>Commands.kt`, rename the extension
   functions, and update the imports to point at the new subsystem.
4. In `RobotContainer.kt`: declare the new subsystem, give it a default command in
   `setDefaultCommands()`, and bind any buttons/triggers to it in `configureButtonBindings()`.
5. Once `ExampleSubsystem`/`ExampleCommands`/the `Example` entry in `RobotMap` are no longer
   referenced anywhere, delete them.

## What AlonLib gives you

Hardware wrappers (`HaMotor`, `HaServo`, `HaLimelight3A`, `HaPinPoint`), a units system (`Length`,
`AngularVelocity`, `Rotation2d` extensions so you're never guessing whether a number is degrees or
radians), PID/feedforward gains, moving-window filters, and SolversLib `Command` extensions. See
[AlonLib's README](https://github.com/alonHamb/AlonLib) for the full rundown. It's declared as a
JitPack dependency in [`TeamCode/build.gradle`](TeamCode/build.gradle) — bump the version there to
pick up library updates.

## Building

```bash
./gradlew :TeamCode:assembleDebug
```
