package org.firstinspires.ftc.teamcode.roadrunner

import alonlib.commands.Command
import alonlib.commands.CommandBase
import alonlib.commands.Subsystem
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import alonlib.math.geometry.Pose2d as AlonLibPose2d
import com.acmerobotics.roadrunner.Pose2d as RoadRunnerPose2d

/**
 * Converts an AlonLib [AlonLibPose2d] to RoadRunner's own [RoadRunnerPose2d]. AlonLib provided this
 * conversion (`alonlib.units.toRoadRunner`) through 1.0.0, but dropped it once it stopped depending
 * on RoadRunner (1.0.1+), so it's ported here instead.
 */
fun AlonLibPose2d.toRoadRunner(): RoadRunnerPose2d = RoadRunnerPose2d(x, y, rotation.radians)

/**
 * Wraps a RoadRunner [Action] (e.g. a trajectory built with `MecanumDrive.actionBuilder(...).build()`)
 * as a [Command], so it can be scheduled as a default/triggered command or combined with other
 * commands instead of only being runnable via [runBlocking].
 *
 * Ported from AlonLib's own `alonlib.commands.ActionCommand`/`asCommand` (through 1.0.0), dropped
 * once the library stopped depending on RoadRunner (1.0.1+).
 */
class ActionCommand(private val action: Action, vararg requirements: Subsystem) : CommandBase() {
    private var finished = false

    init {
        addRequirements(*requirements)
    }

    override fun initialize() {
        finished = false
    }

    override fun execute() {
        val packet = TelemetryPacket()
        finished = !action.run(packet)
        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    override fun isFinished(): Boolean = finished
}

/** @see ActionCommand */
fun Action.asCommand(vararg requirements: Subsystem): Command = ActionCommand(this, *requirements)
