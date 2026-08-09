package org.firstinspires.ftc.teamcode.roadrunner.messages

import com.acmerobotics.roadrunner.Pose2d

class PoseMessage(pose: Pose2d) {
    @JvmField val timestamp: Long = System.nanoTime()
    @JvmField val x: Double = pose.position.x
    @JvmField val y: Double = pose.position.y
    @JvmField val heading: Double = pose.heading.toDouble()
}
