package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotMap
import alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.runBlocking
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS

/**
 * Repeatedly drives DISTANCE inches forward and back so you can dial in [MecanumDrive.Params]'
 * axial/lateral/heading gains from the FTC Dashboard field view while it's running.
 */
@TeleOp(group = TuningOpModes.GROUP)
@Config
class ManualFeedbackTuner : LinearOpMode() {
    companion object {
        @JvmField var DISTANCE = 64.0
    }

    override fun runOpMode() {
        val pinPoint = HaPinPoint(hardwareMap, RobotMap.Drive.PINPOINT_ID, PINPOINT_ODOMETRY_PODS)
        val drive = MecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0))

        waitForStart()

        while (opModeIsActive()) {
            runBlocking(
                drive.actionBuilder(Pose2d(0.0, 0.0, 0.0))
                    .lineToX(DISTANCE)
                    .lineToX(0.0)
                    .build()
            )
        }
    }
}
