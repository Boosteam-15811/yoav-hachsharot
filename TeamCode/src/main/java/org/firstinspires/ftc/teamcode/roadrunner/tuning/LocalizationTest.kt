package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotMap
import org.firstinspires.ftc.teamcode.alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.Drawing
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS


/** Drives with gamepad1 while streaming the pose estimate to telemetry and the dashboard field view. */
@TeleOp(group = TuningOpModes.GROUP)
class LocalizationTest : LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val pinPoint = HaPinPoint(hardwareMap, RobotMap.Drive.PINPOINT_ID, PINPOINT_ODOMETRY_PODS)
        val drive = MecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            drive.setDrivePowers(
                PoseVelocity2d(
                    Vector2d(-gamepad1.left_stick_y.toDouble(), -gamepad1.left_stick_x.toDouble()),
                    -gamepad1.right_stick_x.toDouble()
                )
            )

            drive.updatePoseEstimate()

            val pose = drive.localizer.pose
            telemetry.addData("x", pose.position.x)
            telemetry.addData("y", pose.position.y)
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()))
            telemetry.update()

            val packet = TelemetryPacket()
            packet.fieldOverlay().setStroke("#3F51B5")
            Drawing.drawRobot(packet.fieldOverlay(), pose)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
    }
}
