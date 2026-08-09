package org.firstinspires.ftc.teamcode.opmodes.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.seattlesolvers.solverslib.command.CommandOpMode
import org.firstinspires.ftc.teamcode.RobotContainer
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.units.Alliance

@Autonomous(name = "Main Autonomous", group = "Autonomous")
class MainAutonomous : CommandOpMode() {
    lateinit var hub: LynxModule
    lateinit var robotContainer: RobotContainer

    override fun initialize() {
        hub = hardwareMap.get(LynxModule::class.java, "Control Hub").apply {
            bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.addLine("Robot initializing")
        robotContainer = RobotContainer(
            hardwareMap,
            telemetry,
            gamepad1,
            gamepad2,
            Alliance.Blue,
            TelemetryLevel.Testing
        )
        telemetry.update()

        // Build the autonomous routine here, e.g.:
        // schedule(robotContainer.exampleSubsystem.exampleCommand(0.5).withTimeout(2.0))
    }

    override fun run() {
        hub.clearBulkCache()
        super.run()
        telemetry.update()
    }
}
