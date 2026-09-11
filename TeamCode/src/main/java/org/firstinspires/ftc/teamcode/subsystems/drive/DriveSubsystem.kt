package org.firstinspires.ftc.teamcode.subsystems.drive

import alonlib.TelemetryLevel
import alonlib.commands.SubsystemBase
import alonlib.drives.HaMecanumDrive
import alonlib.hardware.Data
import alonlib.hardware.Data.Motors.Direction.Forward
import alonlib.hardware.motors.HaMotor
import alonlib.hardware.sensors.HaPinPoint
import alonlib.math.geometry.Pose2d
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_X_OFFSET
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_Y_OFFSET

@Config
class DriveSubsystem(
	val hardwareMap: HardwareMap,
	val telemetry: Telemetry,
	val telemetryLevel: TelemetryLevel
) : SubsystemBase() {

	// --- hardware declaration ---
	val frontLeftMotor = HaMotor(hardwareMap, FRONT_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE).apply {
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runningDirection = Forward
		runMode = Data.Motors.RunMode.VelocityControl
	}
	val frontRightMotor = HaMotor(hardwareMap, FRONT_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE).apply {
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runningDirection = Forward
		runMode = Data.Motors.RunMode.VelocityControl
	}
	val backLeftMotor = HaMotor(hardwareMap, BACK_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE).apply {
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runningDirection = Forward
		runMode = Data.Motors.RunMode.VelocityControl
	}
	val backRightMotor = HaMotor(hardwareMap, BACK_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE).apply {
		zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.Float
		runningDirection = Forward
		runMode = Data.Motors.RunMode.VelocityControl
	}
	val pinPoint = HaPinPoint(hardwareMap, PINPOINT_ID, PINPOINT_ODOMETRY_PODS).apply {
		xOffset = PINPOINT_X_OFFSET
		yOffset = PINPOINT_Y_OFFSET
	}

	// --- properties ---

	val drive = HaMecanumDrive(
		frontLeftMotor,
		frontRightMotor,
		backLeftMotor,
		backRightMotor
	)

	// --- operation functions ---
	fun fieldCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
		drive.driveFieldCentric(xSpeed, ySpeed, turnSpeed, pinPoint.heading.degrees)
	}

	fun robotCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
		drive.driveRobotCentric(xSpeed, ySpeed, turnSpeed)
	}

	fun resetPose(pose: Pose2d) {
		pinPoint.position = pose
	}

	fun resetImu() {
		pinPoint.resetIMU()
	}

	// --- telemetry ---
	fun updateTelemetry() {
		when (telemetryLevel) {
			TelemetryLevel.Competition -> {}
			TelemetryLevel.Testing     -> {
				telemetry.addLine("--- drive subsystem ---")
				telemetry.addData("Running Command", super.currentCommand())
				telemetry.addData(
					"pose",
					"x: ${pinPoint.position.x}, y: ${pinPoint.position.y}, heading (deg): ${pinPoint.heading.degrees}"
				)
			}
		}
	}

	// --- periodic function ---
	override fun periodic() {
		updateTelemetry()
	}
}
