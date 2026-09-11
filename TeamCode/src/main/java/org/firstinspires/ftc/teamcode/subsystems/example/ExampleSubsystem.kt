package org.firstinspires.ftc.teamcode.subsystems.example

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Example.EXAMPLE_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Example.EXAMPLE_MOTOR_TYPE
import alonlib.TelemetryLevel
import alonlib.commands.SubsystemBase
import alonlib.hardware.Data
import alonlib.hardware.motors.HaMotor
import alonlib.units.Percentage


/**
 * Template subsystem wrapping a single motor.
 *
 * To add a real mechanism: copy this file, [org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants],
 * and [org.firstinspires.ftc.teamcode.commands.ExampleCommands] into a new package named after the
 * mechanism, rename "Example"/"example" throughout, and add the hardware ID to [RobotMap].
 */
class ExampleSubsystem(
    hardwareMap: HardwareMap,
    var telemetry: Telemetry,
    val telemetryLevel: TelemetryLevel
) : SubsystemBase() {
    // --- hardware declaration ---
    val motor = HaMotor(hardwareMap, EXAMPLE_MOTOR_ID, EXAMPLE_MOTOR_TYPE).apply {
        zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Brake
        runningDirection = Data.Motors.Direction.Forward
        runMode = Data.Motors.RunMode.RawPower
    }

    // --- operation functions ---
    fun setMotorPower(power: Percentage) {
        motor.percentOutput = power
    }

    fun stopMotor() {
        motor.stop()
    }

    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- example subsystem ---")
                telemetry.addData("Running Command", super.currentCommand())
                telemetry.addData("motor power", motor.percentOutput)
            }
        }
    }

    // --- periodic function ---
    override fun periodic() {
        updateTelemetry()
    }
}
