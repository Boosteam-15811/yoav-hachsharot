package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotContainer
import alonlib.TelemetryLevel
import alonlib.commands.CommandOpMode
import alonlib.units.Alliance

@TeleOp(name = "Red Main Teleop", group = "Teleop")
class RedMainTeleop : CommandOpMode() {
    lateinit var hub: LynxModule


    override fun initialize() {
        hub = hardwareMap.get(LynxModule::class.java, "Control Hub").apply {
            bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 50
        telemetry.addLine("Robot initializing")
        RobotContainer(
            hardwareMap,
            telemetry,
            gamepad1,
            gamepad2,
            Alliance.Red,
            TelemetryLevel.Testing
        )
        telemetry.update()
    }

    override fun run() {
        hub.clearBulkCache()
        super.run()
        telemetry.update()
    }
}
