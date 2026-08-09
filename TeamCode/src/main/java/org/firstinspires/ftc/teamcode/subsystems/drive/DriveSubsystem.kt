package org.firstinspires.ftc.teamcode.subsystems.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.command.SubsystemBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Drive.PINPOINT_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS

/**
 * Drives the robot via RoadRunner's [MecanumDrive] (see [mecanumDrive]), which owns the
 * kinematics, pose estimate (from a goBILDA Pinpoint), and PID/feedforward trajectory following.
 *
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
    val pinPoint = HaPinPoint(hardwareMap, PINPOINT_ID, PINPOINT_ODOMETRY_PODS)

    // --- functional properties ---
    val mecanumDrive = MecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0))

    // --- operation functions ---
    fun fieldCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        val heading = mecanumDrive.localizer.pose.heading
        val robotRelative = Rotation2d.fromDouble(-heading.log()).times(Vector2d(xSpeed, ySpeed))
        mecanumDrive.setDrivePowers(PoseVelocity2d(robotRelative, turnSpeed))
        mecanumDrive.updatePoseEstimate()
    }

    fun robotCentricDrive(xSpeed: Double, ySpeed: Double, turnSpeed: Double) {
        mecanumDrive.setDrivePowers(PoseVelocity2d(Vector2d(xSpeed, ySpeed), turnSpeed))
        mecanumDrive.updatePoseEstimate()
    }

    fun resetPose(pose: Pose2d) {
        mecanumDrive.localizer.pose = pose
    }

    fun resetImu() {
        val current = mecanumDrive.localizer.pose
        resetPose(Pose2d(current.position.x, current.position.y, 0.0))
    }

    /** Builds a RoadRunner trajectory starting from the current pose estimate. */
    fun actionBuilder(): TrajectoryActionBuilder = mecanumDrive.actionBuilder(mecanumDrive.localizer.pose)

    // --- telemetry ---
    fun updateTelemetry() {
        when (telemetryLevel) {
            TelemetryLevel.Competition -> {}
            TelemetryLevel.Testing -> {
                telemetry.addLine("--- drive subsystem ---")
                telemetry.addData("Running Command", super.currentCommand)
                val pose = mecanumDrive.localizer.pose
                telemetry.addData(
                    "pose",
                    "x: ${pose.position.x}, y: ${pose.position.y}, heading (deg): ${Math.toDegrees(pose.heading.toDouble())}"
                )
            }
        }
    }

    // --- periodic function ---
    override fun periodic() {
        updateTelemetry()
    }
}
