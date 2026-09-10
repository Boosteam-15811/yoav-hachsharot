package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import alonlib.TelemetryLevel
import alonlib.commands.SubsystemBase
import alonlib.drives.HaMecanumDrive
import alonlib.hardware.Data
import alonlib.hardware.Data.Motors.Direction.Forward
import alonlib.hardware.motors.HaMotor
import alonlib.hardware.sensors.HaPinPoint
import alonlib.math.geometry.Pose2d
import alonlib.units.degrees
import org.firstinspires.ftc.teamcode.roadrunner.toRoadRunner
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_X_OFFSET
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_Y_OFFSET
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive as RoadrunnerMecanumDrive

/**
 * [RoadrunnerMecanumDrive.PARAMS] defaults to placeholders -- tune it first via
 * [org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes] or trajectories won't track
 * accurately.
 */
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
    val roadRunnerDrive =
        RoadrunnerMecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0.degrees).toRoadRunner())
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
        roadRunnerDrive.setDrivePowers(PoseVelocity2d(Vector2d(xSpeed, ySpeed), turnSpeed))
        roadRunnerDrive.updatePoseEstimate()
    }

    fun resetPose(pose: Pose2d) {
        roadRunnerDrive.localizer.pose = pose.toRoadRunner()
        pinPoint.position = pose
    }

    fun resetImu() {
        pinPoint.resetIMU()
    }

    /** Builds a RoadRunner trajectory starting from the current pose estimate. */
    fun actionBuilder(): TrajectoryActionBuilder =
        roadRunnerDrive.actionBuilder(roadRunnerDrive.localizer.pose)

    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
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
