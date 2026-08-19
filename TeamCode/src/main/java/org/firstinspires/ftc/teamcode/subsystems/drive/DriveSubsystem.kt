package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import com.seattlesolvers.solverslib.drivebase.MecanumDrive
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.BACK_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.DRIVE_MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_LEFT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.FRONT_RIGHT_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS

/**
 * [MecanumDrive.PARAMS] defaults to placeholders -- tune it first via
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
    val frontLeftMotor = HaMotor(hardwareMap, FRONT_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val frontRightMotor = HaMotor(hardwareMap, FRONT_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backLeftMotor = HaMotor(hardwareMap, BACK_LEFT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val backRightMotor = HaMotor(hardwareMap, BACK_RIGHT_MOTOR_ID, DRIVE_MOTOR_TYPE)
    val pinPoint = HaPinPoint(hardwareMap, PINPOINT_ID, PINPOINT_ODOMETRY_PODS)

    // --- functional properties ---
    val roadRunnerDrive = org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0))
    val drive = MecanumDrive(frontLeftMotor.motor, frontRightMotor.motor, backLeftMotor.motor, backRightMotor.motor)

    // --- operation functions ---
    fun fieldCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        val heading = roadRunnerDrive.localizer.pose.heading
        val robotRelative = Rotation2d.fromDouble(-heading.log()).times(Vector2d(xSpeed, ySpeed))
        roadRunnerDrive.setDrivePowers(PoseVelocity2d(robotRelative, turnSpeed))
        roadRunnerDrive.updatePoseEstimate()
    }

    fun robotCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        roadRunnerDrive.setDrivePowers(PoseVelocity2d(Vector2d(xSpeed, ySpeed), turnSpeed))
        roadRunnerDrive.updatePoseEstimate()
    }

    fun resetPose(pose: Pose2d) {
        roadRunnerDrive.localizer.pose = pose
    }

    fun resetImu() {
        val current = roadRunnerDrive.localizer.pose
        resetPose(Pose2d(current.position.x, current.position.y, 0.0))
    }

    /** Builds a RoadRunner trajectory starting from the current pose estimate. */
    fun actionBuilder(): TrajectoryActionBuilder = roadRunnerDrive.actionBuilder(roadRunnerDrive.localizer.pose)

    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- drive subsystem ---")
                telemetry.addData("Running Command", super.currentCommand)
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
