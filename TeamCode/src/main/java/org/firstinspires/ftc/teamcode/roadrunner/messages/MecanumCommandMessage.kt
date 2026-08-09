package org.firstinspires.ftc.teamcode.roadrunner.messages

class MecanumCommandMessage(
    @JvmField val voltage: Double,
    @JvmField val leftFrontPower: Double,
    @JvmField val leftBackPower: Double,
    @JvmField val rightBackPower: Double,
    @JvmField val rightFrontPower: Double
) {
    @JvmField val timestamp: Long = System.nanoTime()
}
