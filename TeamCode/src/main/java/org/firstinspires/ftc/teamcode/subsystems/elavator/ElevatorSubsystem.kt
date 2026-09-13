package org.firstinspires.ftc.teamcode.subsystems.elavator

import alonlib.TelemetryLevel
import alonlib.commands.SubsystemBase
import alonlib.hardware.Data
import alonlib.hardware.motors.HaMotor
import alonlib.units.Angle
import alonlib.units.amps
import alonlib.units.degrees
import alonlib.units.volts
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Elevator.RIGHT_MOTOR_ID

class ElevatorSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) :
	SubsystemBase() {

	// --- hardware declarations ---
	val leftMotor = HaMotor(hardwareMap, LEFT_MOTOR_ID, MOTOR_TYPE).apply {
		runningDirection = Data.Motors.Direction.Forward
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runMode = Data.Motors.RunMode.PositionControl
	}
	val rightMotor = HaMotor(hardwareMap, RIGHT_MOTOR_ID, MOTOR_TYPE, leftMotor).apply {
		runningDirection = Data.Motors.Direction.Reverse
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runMode = Data.Motors.RunMode.PositionControl
	}

	// --- getters and setters ---
	val currentPosition: Angle
		get() = rightMotor.angularPosition
	var positionSetpoint: Angle
		get() = rightMotor.angularPosition
		set(setpoint) {
			rightMotor.angularPosition = setpoint
		}

	var maxPosition: Angle = 200.degrees
	val isAtMaxLimit: Boolean
		get() {
			return currentPosition > maxPosition
		}
	var minPosition: Angle = 5.degrees
	val isAtMinLimit: Boolean
		get() = currentPosition < minPosition

	fun homing() {
		while (rightMotor.current <= 7.amps) {
			rightMotor.voltage = 6.volts
		}
		rightMotor.maximumAngle = currentPosition
	}

	fun telemetryUpdate() {
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
