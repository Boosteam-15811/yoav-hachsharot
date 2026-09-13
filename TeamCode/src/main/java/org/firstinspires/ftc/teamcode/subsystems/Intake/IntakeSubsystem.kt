package org.firstinspires.ftc.teamcode.subsystems.Intake

import alonlib.TelemetryLevel
import alonlib.commands.SubsystemBase
import alonlib.hardware.Data
import alonlib.hardware.motors.HaMotor
import alonlib.hardware.servos.HaServo
import alonlib.units.Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Intake.EXTENSION_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.LEFT_ANGLE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.LEFT_INTAKE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Intake.RIGHT_ANGLE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.RIGHT_INTAKE_SERVO_ID
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.MAX_ANGLE
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.MIN_ANGLE

class IntakeSubsystem(hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel) :
	SubsystemBase() {

	// --- hardware declarations ---
	val intakeMotor = HaMotor(hardwareMap, EXTENSION_MOTOR_ID, MOTOR_TYPE).apply {
		runningDirection = Data.Motors.Direction.Forward
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runMode = Data.Motors.RunMode.PositionControl
	}
	val leftAngleServo = HaServo(hardwareMap, LEFT_ANGLE_SERVO_ID, Data.Servos.Mode.FullRange, Data.Servos.Type.AxonMax).apply {
		minPosition = MIN_ANGLE
		maxPosition = MAX_ANGLE
	}
	val rightAngleServo = HaServo(hardwareMap, RIGHT_ANGLE_SERVO_ID, Data.Servos.Mode.FullRange, Data.Servos.Type.AxonMax, leftAngleServo).apply {
		minPosition = MIN_ANGLE
		maxPosition = MAX_ANGLE
	}
	val rightIntakeServo = HaServo(hardwareMap, RIGHT_INTAKE_SERVO_ID, Data.Servos.Mode.Cr, Data.Servos.Type.AxonMax)
	val leftIntakeServo = HaServo(hardwareMap, LEFT_INTAKE_SERVO_ID, Data.Servos.Mode.Cr, Data.Servos.Type.AxonMax)

	val currentPosition: Angle
		get() = rightAngleServo.position

	var currentPositionSetpoint: Angle
		get() = rightAngleServo.position
		set(setpoint) {
			rightAngleServo.position = setpoint
		}

	var maxPosition: Angle = MAX_ANGLE

	val isAtMaxPosition: Boolean
		get() = currentPosition >= MAX_ANGLE

}
