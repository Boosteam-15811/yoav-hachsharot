package org.firstinspires.ftc.teamcode.subsystems.drive

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver

/**
 * Pod-offset and encoder-direction configuration for the Pinpoint lives in
 * [org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer] (its `PARAMS.parYTicks`/
 * `perpXTicks`, tuned via [org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes], and
 * its `initialParDirection`/`initialPerpDirection`), not here.
 */
object DriveConstants {
    val PINPOINT_ODOMETRY_PODS = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
}
