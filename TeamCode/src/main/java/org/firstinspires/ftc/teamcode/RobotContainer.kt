package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.Robot
import com.seattlesolvers.solverslib.command.button.GamepadButton
import com.seattlesolvers.solverslib.gamepad.GamepadEx
import com.seattlesolvers.solverslib.gamepad.GamepadKeys
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance
import org.firstinspires.ftc.teamcode.commands.defaultExampleCommand
import org.firstinspires.ftc.teamcode.commands.runAtFullPowerCommand
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
) : Robot() {
    val controllerA = GamepadEx(gamepad1)
    val controllerB = GamepadEx(gamepad2)

    // --- Subsystem declaration ---
    val exampleSubsystem = ExampleSubsystem(hardwareMap, telemetry, telemetryLevel)

    // --- init functions ---
    init {
        configureButtonBindings()
        setDefaultCommands()
    }

    fun configureButtonBindings() {
        with(controllerA) {
            GamepadButton(this, GamepadKeys.Button.A).whenPressed(exampleSubsystem.runAtFullPowerCommand())
        }
        with(controllerB) {

        }
    }

    fun setDefaultCommands() {
        exampleSubsystem.defaultCommand = exampleSubsystem.defaultExampleCommand()
    }
}
