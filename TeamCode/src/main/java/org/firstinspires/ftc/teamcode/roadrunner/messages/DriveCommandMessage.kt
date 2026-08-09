package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.Time

class DriveCommandMessage(poseVelocity: PoseVelocity2dDual<Time>) {
    @JvmField val timestamp: Long = System.nanoTime()
    @JvmField val forwardVelocity: Double = poseVelocity.linearVel.x.get(0)
    @JvmField val forwardAcceleration: Double = poseVelocity.linearVel.x.get(1)
    @JvmField val lateralVelocity: Double = poseVelocity.linearVel.y.get(0)
    @JvmField val lateralAcceleration: Double = poseVelocity.linearVel.y.get(1)
    @JvmField val angularVelocity: Double = poseVelocity.angVel.get(0)
    @JvmField val angularAcceleration: Double = poseVelocity.angVel.get(1)
}
