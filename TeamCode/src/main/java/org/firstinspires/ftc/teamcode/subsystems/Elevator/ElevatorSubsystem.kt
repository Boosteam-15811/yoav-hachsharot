package org.firstinspires.ftc.teamcode.subsystems.Elevator

import com.qualcomm.robotcore.hardware.HardwareMap
import emulator.hardware.ZeroPowerBehavior
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.commands.SubsystemBase
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.Length
import org.firstinspires.ftc.teamcode.alonlib.units.amps
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.alonlib.units.degrees
import org.firstinspires.ftc.teamcode.alonlib.units.percent
import org.firstinspires.ftc.teamcode.alonlib.units.volts

class ElevatorSubsystem(val hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel):
    SubsystemBase()
{
    // --- hardware declarations ---
    val leftMotor = HaMotor(hardwareMap, LEFT_MOTOR_ID, MOTOR_TYPE).apply {
        runningDirection = Data.Motors.Direction.FORWARD
        zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.FLOAT
        runMode = Data.Motors.RunMode.POSITION_CONTROL
    }
    val rightMotor = HaMotor(hardwareMap, RIGHT_MOTOR_ID, MOTOR_TYPE,leftMotor).apply {
        runningDirection = Data.Motors.Direction.REVERSE
        zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.FLOAT
        runMode = Data.Motors.RunMode.POSITION_CONTROL
    }

    // --- getters and setters ---
    val currentPosition : Rotation2d
        get() = rightMotor.position

    var positionSetpoint : Rotation2d
        get() = rightMotor.position
        set(setpoint) {
            rightMotor.position = setpoint
        }

    var maxPosition : Rotation2d = 200.degrees
    val isAtMaxLimit : Boolean
        get() {
           return currentPosition > maxPosition
        }
    var minPosition : Rotation2d = 5.degrees
    val  isAtMinLimit : Boolean
        get() = currentPosition < minPosition

    fun homing()
    {
        while (rightMotor.current <= 7.amps)
        {
            rightMotor.voltage = 6.volts
        }
        rightMotor.maximumPosition = currentPosition
    }

    fun telemetryUpdate()
    {
        telemetry.addLine("--- Elevator Subsystem ---")
        telemetry.addLine("Running Command: ${super.currentCommand()}")
        telemetry.addLine("Current Position: $currentPosition")
        telemetry.addLine("Position SetPoint: $positionSetpoint")
        telemetry.addLine("Master Motor Power: ${rightMotor.percentOutput.asFraction}")
        telemetry.addLine("Slave Motor Power: ${leftMotor.percentOutput.asFraction}")
        telemetry.addLine("is At Max Limit: $isAtMaxLimit")
        telemetry.addLine("is At Min Limit: $isAtMinLimit")
    }

    override fun periodic() {
        rightMotor.update()
        leftMotor.update()
        telemetryUpdate()
    }
}