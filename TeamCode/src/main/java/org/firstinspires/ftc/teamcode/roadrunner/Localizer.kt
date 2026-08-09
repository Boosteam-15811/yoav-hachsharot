package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d

/** Interface for localization methods. */
interface Localizer {
    /**
     * The current pose estimate. Reading this does NOT update the estimate; call [update] first.
     */
    var pose: Pose2d

    /**
     * Updates the pose estimate.
     * @return the current velocity estimate
     */
    fun update(): PoseVelocity2d
}
