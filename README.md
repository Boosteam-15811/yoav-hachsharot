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
  - `opmodes/teleop/MainTeleop.kt` and `opmodes/autonomous/MainAutonomous.kt` — minimal OpModes
    that construct a `RobotContainer` and hand control to the command scheduler.

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
