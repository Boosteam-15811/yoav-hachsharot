package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.DefaultCommand
import org.firstinspires.ftc.teamcode.commands.defaultExampleCommand
import org.firstinspires.ftc.teamcode.commands.driveFieldCentricCommand
import org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveSubsystem
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

/*
  this is the robot container class it contains all of your robots subsystems and the controllers you are going to use

  to add a new subsystem: declare it below, wire its default command in [setDefaultCommands],
  and bind any buttons/triggers to it in [configureButtonBindings]
 */
class RobotContainer(
    hardwareMap: HardwareMap,
    telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad,
    val alliance: Alliance,
    telemetryLevel: TelemetryLevel
) {
    val controllerA: GamepadEx = GamepadEx(gamepad1)
    val controllerB: GamepadEx = GamepadEx(gamepad2)

    // --- Subsystem declaration ---
    val driveSubsystem = DriveSubsystem(hardwareMap, telemetry, telemetryLevel)
    val exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry, telemetryLevel)
    val elevatorSubsystem = ElevatorSubsystem(hardwareMap, telemetry, telemetryLevel)
    val intakeSubsystem = IntakeSubsystem(hardwareMap, telemetry,telemetryLevel)

    // --- init functions ---
    init {
        configureButtonBindings()
        setDefaultCommands()
    }

    fun configureButtonBindings() {


    }

    fun setDefaultCommands() {
        driveSubsystem.setDefaultCommand(
            driveSubsystem.driveFieldCentricCommand(
                { controllerA.leftX },
                { controllerA.leftY }
            ) { controllerA.rightX })
        exampleSubsystem.setDefaultCommand(exampleSubsystem.defaultExampleCommand())
        elevatorSubsystem.setDefaultCommand(elevatorSubsystem.DefaultCommand())

    }
}
