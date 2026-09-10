package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import alonlib.hardware.sensors.HaPinPoint

/**
 * Wraps an already-constructed [HaPinPoint] as a RoadRunner [Localizer], so the same physical
 * Pinpoint device can be shared with other consumers (e.g. a vision/localizer subsystem) instead
 * of this claiming the hardware a second time.
 *
 * Ported from the official RoadRunner FTC quickstart's PinpointLocalizer.
 */
@Config
class PinpointLocalizer(
    private val haPinPoint: HaPinPoint,
    inPerTick: Double,
    initialPose: Pose2d
) : Localizer {
    class Params {
        @JvmField var parYTicks = 0.0 // y position of the parallel encoder (in tick units)
        @JvmField var perpXTicks = 0.0 // x position of the perpendicular encoder (in tick units)
    }

    companion object {
        @JvmField var PARAMS = Params()
    }

    val driver: GoBildaPinpointDriver get() = haPinPoint.driver

    // TODO: reverse encoder directions if needed
    val initialParDirection: GoBildaPinpointDriver.EncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD
    val initialPerpDirection: GoBildaPinpointDriver.EncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD

    private var txWorldPinpoint: Pose2d
    private var txPinpointRobot = Pose2d(0.0, 0.0, 0.0)

    init {
        val mmPerTick = inPerTick * 25.4
        driver.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM)
        driver.setOffsets(mmPerTick * PARAMS.parYTicks, mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM)

        driver.setEncoderDirections(initialParDirection, initialPerpDirection)

        driver.resetPosAndIMU()

        txWorldPinpoint = initialPose
    }

    override var pose: Pose2d
        get() = txWorldPinpoint.times(txPinpointRobot)
        set(value) {
            txWorldPinpoint = value.times(txPinpointRobot.inverse())
        }

    override fun update(): PoseVelocity2d {
        driver.update()
        if (driver.deviceStatus == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = Pose2d(
                driver.getPosX(DistanceUnit.INCH),
                driver.getPosY(DistanceUnit.INCH),
                driver.getHeading(UnnormalizedAngleUnit.RADIANS)
            )
            val worldVelocity = Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH))
            val robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity)

            return PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS))
        }
        return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
    }
}
