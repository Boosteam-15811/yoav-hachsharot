package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

/** Monotonic clock in seconds, matching what RoadRunner's own `Actions.now()` provides. */
fun now(): Double = System.nanoTime() * 1e-9

/**
 * Runs an [Action] to completion on the current thread, forwarding its telemetry packets to FTC
 * Dashboard each cycle -- equivalent to RoadRunner's own `Actions.runBlocking`, used by the
 * tuning LinearOpModes ([tuning.ManualFeedbackTuner], [tuning.SplineTest]). Regular autonomous
 * code should prefer scheduling the action as a command via
 * [org.firstinspires.ftc.teamcode.alonlib.commands.asCommand] instead.
 */
fun runBlocking(action: Action) {
    val dashboard = FtcDashboard.getInstance()
    while (true) {
        val packet = TelemetryPacket()
        val running = action.run(packet)
        dashboard.sendTelemetryPacket(packet)
        if (!running) break
    }
}
