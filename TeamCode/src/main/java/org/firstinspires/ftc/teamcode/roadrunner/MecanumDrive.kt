package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.AngularVelConstraint
import com.acmerobotics.roadrunner.HolonomicController
import com.acmerobotics.roadrunner.MecanumKinematics
import com.acmerobotics.roadrunner.MinVelConstraint
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.PoseVelocity2dDual
import com.acmerobotics.roadrunner.ProfileAccelConstraint
import com.acmerobotics.roadrunner.ProfileParams
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.TimeTrajectory
import com.acmerobotics.roadrunner.TimeTurn
import com.acmerobotics.roadrunner.TrajectoryActionBuilder
import com.acmerobotics.roadrunner.TrajectoryBuilderParams
import com.acmerobotics.roadrunner.TurnConstraints
import com.acmerobotics.roadrunner.VelConstraint
import com.acmerobotics.roadrunner.ftc.DownsampledWriter
import com.acmerobotics.roadrunner.ftc.FlightRecorder
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.RobotMap
import alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.messages.DriveCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.MecanumCommandMessage
import org.firstinspires.ftc.teamcode.roadrunner.messages.PoseMessage
import java.util.LinkedList
import kotlin.math.PI
import kotlin.math.ceil
import kotlin.math.max

/**
 * RoadRunner's Mecanum drive controller: kinematics, [PinpointLocalizer]-based pose tracking, and
 * PID/feedforward trajectory following.
 *
 * Ported from the official RoadRunner FTC quickstart, simplified to Pinpoint-only localization --
 * the encoder/hub-IMU `DriveLocalizer` from the reference implementation was dropped since we
 * always use a goBILDA Pinpoint (see [PinpointLocalizer]).
 *
 * [PARAMS] is tuned via the opmodes in `roadrunner.tuning`
 * (see [org.firstinspires.ftc.teamcode.roadrunner.tuning.TuningOpModes]); the defaults below are
 * placeholders and MUST be re-tuned for your robot before trajectories will track accurately.
 */
@Config
class MecanumDrive(hardwareMap: HardwareMap, pinPoint: HaPinPoint, pose: Pose2d) {
    class Params {
        // drive model parameters
        @JvmField var inPerTick = 1.0
        @JvmField var lateralInPerTick = inPerTick
        @JvmField var trackWidthTicks = 0.0

        // feedforward parameters (in tick units)
        @JvmField var kS = 0.0
        @JvmField var kV = 0.0
        @JvmField var kA = 0.0

        // path profile parameters (in inches)
        @JvmField var maxWheelVel = 50.0
        @JvmField var minProfileAccel = -30.0
        @JvmField var maxProfileAccel = 50.0

        // turn profile parameters (in radians)
        @JvmField var maxAngVel = PI // shared with path
        @JvmField var maxAngAccel = PI

        // path controller gains
        @JvmField var axialGain = 0.0
        @JvmField var lateralGain = 0.0
        @JvmField var headingGain = 0.0 // shared with turn

        @JvmField var axialVelGain = 0.0
        @JvmField var lateralVelGain = 0.0
        @JvmField var headingVelGain = 0.0 // shared with turn
    }

    companion object {
        @JvmField var PARAMS = Params()
    }

    val kinematics = MecanumKinematics(
        PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick
    )

    val defaultTurnConstraints = TurnConstraints(PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel)
    val defaultVelConstraint: VelConstraint = MinVelConstraint(
        listOf(
            kinematics.WheelVelConstraint(PARAMS.maxWheelVel),
            AngularVelConstraint(PARAMS.maxAngVel)
        )
    )
    val defaultAccelConstraint: AccelConstraint =
        ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel)

    // TODO: make sure your config has motors with these names (or change them in RobotMap.Drive)
    val leftFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, RobotMap.Drive.FRONT_LEFT_MOTOR_ID)
    val leftBack: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, RobotMap.Drive.BACK_LEFT_MOTOR_ID)
    val rightBack: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, RobotMap.Drive.BACK_RIGHT_MOTOR_ID)
    val rightFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, RobotMap.Drive.FRONT_RIGHT_MOTOR_ID)

    val voltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    val localizer: Localizer = PinpointLocalizer(pinPoint, PARAMS.inPerTick, pose)
    private val poseHistory = LinkedList<Pose2d>()

    private val estimatedPoseWriter = DownsampledWriter("ESTIMATED_POSE", 50_000_000)
    private val targetPoseWriter = DownsampledWriter("TARGET_POSE", 50_000_000)
    private val driveCommandWriter = DownsampledWriter("DRIVE_COMMAND", 50_000_000)
    private val mecanumCommandWriter = DownsampledWriter("MECANUM_COMMAND", 50_000_000)

    init {
        leftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        leftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        rightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        // TODO: reverse motor directions if needed
        //   leftFront.direction = DcMotorSimple.Direction.REVERSE

        FlightRecorder.write("MECANUM_PARAMS", PARAMS)
    }

    fun setDrivePowers(powers: PoseVelocity2d) {
        val wheelVels = MecanumKinematics(1.0).inverse(PoseVelocity2dDual.constant<Time>(powers, 1))

        var maxPowerMag = 1.0
        for (power in wheelVels.all()) {
            maxPowerMag = max(maxPowerMag, power.value())
        }

        leftFront.power = wheelVels.leftFront.get(0) / maxPowerMag
        leftBack.power = wheelVels.leftBack.get(0) / maxPowerMag
        rightBack.power = wheelVels.rightBack.get(0) / maxPowerMag
        rightFront.power = wheelVels.rightFront.get(0) / maxPowerMag
    }

    inner class FollowTrajectoryAction(val timeTrajectory: TimeTrajectory) : Action {
        private var beginTs = -1.0
        private val xPoints: DoubleArray
        private val yPoints: DoubleArray

        init {
            val length = timeTrajectory.path.length()
            val n = max(2, ceil(length / 2).toInt())
            // n evenly spaced samples from 0 to length, inclusive
            val disps = DoubleArray(n) { i -> length * i / (n - 1) }
            xPoints = DoubleArray(disps.size)
            yPoints = DoubleArray(disps.size)
            for (i in disps.indices) {
                val p = timeTrajectory.path.get(disps[i], 1).value()
                xPoints[i] = p.position.x
                yPoints[i] = p.position.y
            }
        }

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= timeTrajectory.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = timeTrajectory.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage

            val feedforward = MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick)
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower)
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            p.put("x", localizer.pose.position.x)
            p.put("y", localizer.pose.position.y)
            p.put("heading (deg)", Math.toDegrees(localizer.pose.heading.toDouble()))

            val error = txWorldTarget.value().minusExp(localizer.pose)
            p.put("xError", error.position.x)
            p.put("yError", error.position.y)
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()))

            // only draw when active; only one drive action should be active at a time
            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.pose)

            c.setStroke("#4CAF50FF")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#4CAF507A")
            c.setStrokeWidth(1)
            c.strokePolyline(xPoints, yPoints)
        }
    }

    inner class TurnAction(private val turn: TimeTurn) : Action {
        private var beginTs = -1.0

        override fun run(p: TelemetryPacket): Boolean {
            val t: Double
            if (beginTs < 0) {
                beginTs = now()
                t = 0.0
            } else {
                t = now() - beginTs
            }

            if (t >= turn.duration) {
                leftFront.power = 0.0
                leftBack.power = 0.0
                rightBack.power = 0.0
                rightFront.power = 0.0

                return false
            }

            val txWorldTarget = turn.get(t)
            targetPoseWriter.write(PoseMessage(txWorldTarget.value()))

            val robotVelRobot = updatePoseEstimate()

            val command = HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.pose, robotVelRobot)
            driveCommandWriter.write(DriveCommandMessage(command))

            val wheelVels = kinematics.inverse(command)
            val voltage = voltageSensor.voltage
            val feedforward = MotorFeedforward(PARAMS.kS, PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick)
            val leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage
            val leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage
            val rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage
            val rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage
            mecanumCommandWriter.write(
                MecanumCommandMessage(voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower)
            )

            leftFront.power = leftFrontPower
            leftBack.power = leftBackPower
            rightBack.power = rightBackPower
            rightFront.power = rightFrontPower

            val c = p.fieldOverlay()
            drawPoseHistory(c)

            c.setStroke("#4CAF50")
            Drawing.drawRobot(c, txWorldTarget.value())

            c.setStroke("#3F51B5")
            Drawing.drawRobot(c, localizer.pose)

            c.setStroke("#7C4DFFFF")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)

            return true
        }

        override fun preview(c: Canvas) {
            c.setStroke("#7C4DFF7A")
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2.0)
        }
    }

    fun updatePoseEstimate(): PoseVelocity2d {
        val vel = localizer.update()
        poseHistory.add(localizer.pose)

        while (poseHistory.size > 100) {
            poseHistory.removeFirst()
        }

        estimatedPoseWriter.write(PoseMessage(localizer.pose))

        return vel
    }

    private fun drawPoseHistory(c: Canvas) {
        val xPoints = DoubleArray(poseHistory.size)
        val yPoints = DoubleArray(poseHistory.size)

        var i = 0
        for (t in poseHistory) {
            xPoints[i] = t.position.x
            yPoints[i] = t.position.y
            i++
        }

        c.setStrokeWidth(1)
        c.setStroke("#3F51B5")
        c.strokePolyline(xPoints, yPoints)
    }

    fun actionBuilder(beginPose: Pose2d): TrajectoryActionBuilder {
        return TrajectoryActionBuilder(
            { turn -> TurnAction(turn) },
            { t -> FollowTrajectoryAction(t) },
            TrajectoryBuilderParams(
                1e-6,
                ProfileParams(0.25, 0.1, 1e-2)
            ),
            beginPose, 0.0,
            defaultTurnConstraints,
            defaultVelConstraint, defaultAccelConstraint
        )
    }
}
