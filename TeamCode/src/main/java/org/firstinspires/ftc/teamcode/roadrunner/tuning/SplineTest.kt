package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.RobotMap
import alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import kotlin.math.PI

/** Runs a two-segment spline as a smoke test that trajectory following works end to end. */
@Autonomous(group = TuningOpModes.GROUP)
class SplineTest : LinearOpMode() {
    override fun runOpMode() {
        val beginPose = Pose2d(0.0, 0.0, 0.0)
        val pinPoint = HaPinPoint(hardwareMap, RobotMap.Drive.PINPOINT_ID, PINPOINT_ODOMETRY_PODS)
        val drive = MecanumDrive(hardwareMap, pinPoint, beginPose)

        waitForStart()

        runBlocking(
            drive.actionBuilder(beginPose)
                .splineTo(Vector2d(30.0, 30.0), PI / 2)
                .splineTo(Vector2d(0.0, 60.0), PI)
                .build()
        )
    }
}
