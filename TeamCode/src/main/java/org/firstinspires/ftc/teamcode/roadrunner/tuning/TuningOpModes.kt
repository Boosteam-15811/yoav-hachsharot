package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig
import com.acmerobotics.roadrunner.MotorFeedforward
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.ftc.AngularRampLogger
import com.acmerobotics.roadrunner.ftc.DriveType
import com.acmerobotics.roadrunner.ftc.DriveView
import com.acmerobotics.roadrunner.ftc.DriveViewFactory
import com.acmerobotics.roadrunner.ftc.EncoderGroup
import com.acmerobotics.roadrunner.ftc.EncoderRef
import com.acmerobotics.roadrunner.ftc.ForwardPushTest
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger
import com.acmerobotics.roadrunner.ftc.LateralPushTest
import com.acmerobotics.roadrunner.ftc.LateralRampLogger
import com.acmerobotics.roadrunner.ftc.LazyImu
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup
import com.acmerobotics.roadrunner.ftc.PinpointIMU
import com.acmerobotics.roadrunner.ftc.PinpointView
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta
import org.firstinspires.ftc.teamcode.RobotMap
import alonlib.hardware.sensors.HaPinPoint
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.PINPOINT_ODOMETRY_PODS
import java.util.Arrays

/**
 * Registers the RoadRunner FTC quickstart's tuning opmodes (drive-direction debugger, forward/
 * lateral/angular ramp loggers, manual feedforward tuner) plus our own [ManualFeedbackTuner],
 * [LocalizationTest], and [SplineTest].
 *
 * Simplified from the official quickstart's version to the Mecanum+Pinpoint-only path, since
 * that's the only drive/localizer combination this project uses.
 *
 * Run these from the Driver Station in the order described at https://rr.brott.dev/docs/v1-0/tuning/ :
 * MecanumMotorDirectionDebugger -> ForwardPushTest/LateralPushTest -> ForwardRampLogger/
 * LateralRampLogger/AngularRampLogger (paste the logged data into the RR tuning webapp) ->
 * ManualFeedforwardTuner -> ManualFeedbackTuner.
 */
object TuningOpModes {
    const val GROUP = "quickstart"
    const val DISABLED = false

    private fun metaForClass(cls: Class<out OpMode>): OpModeMeta =
        OpModeMeta.Builder()
            .setName(cls.simpleName)
            .setGroup(GROUP)
            .setFlavor(OpModeMeta.Flavor.TELEOP)
            .build()

    private fun makePinpointView(pl: PinpointLocalizer): PinpointView =
        object : PinpointView {
            private var parGoBildaDirection = pl.initialParDirection
            private var perpGoBildaDirection = pl.initialPerpDirection

            override fun update() {
                pl.driver.update()
            }

            override fun getParEncoderPosition(): Int = pl.driver.encoderX

            override fun getPerpEncoderPosition(): Int = pl.driver.encoderY

            override fun getHeadingVelocity(unit: UnnormalizedAngleUnit): Float =
                pl.driver.getHeadingVelocity(unit).toFloat()

            override var parDirection: DcMotorSimple.Direction
                get() = if (parGoBildaDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    DcMotorSimple.Direction.FORWARD
                else
                    DcMotorSimple.Direction.REVERSE
                set(value) {
                    parGoBildaDirection = if (value == DcMotorSimple.Direction.FORWARD)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                    else
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                    pl.driver.setEncoderDirections(parGoBildaDirection, perpGoBildaDirection)
                }

            override var perpDirection: DcMotorSimple.Direction
                get() = if (perpGoBildaDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    DcMotorSimple.Direction.FORWARD
                else
                    DcMotorSimple.Direction.REVERSE
                set(value) {
                    perpGoBildaDirection = if (value == DcMotorSimple.Direction.FORWARD)
                        GoBildaPinpointDriver.EncoderDirection.FORWARD
                    else
                        GoBildaPinpointDriver.EncoderDirection.REVERSED
                    pl.driver.setEncoderDirections(parGoBildaDirection, perpGoBildaDirection)
                }
        }

    @JvmStatic
    @OpModeRegistrar
    fun register(manager: OpModeManager) {
        if (DISABLED) return

        val dvf = object : DriveViewFactory {
            override fun make(hardwareMap: com.qualcomm.robotcore.hardware.HardwareMap): DriveView {
                val pinPoint = HaPinPoint(hardwareMap, RobotMap.Drive.PINPOINT_ID, PINPOINT_ODOMETRY_PODS)
                val md = MecanumDrive(hardwareMap, pinPoint, Pose2d(0.0, 0.0, 0.0))

                val pv = makePinpointView(md.localizer as PinpointLocalizer)
                val encoderGroups: List<EncoderGroup> = listOf(PinpointEncoderGroup(pv))
                val parEncs = listOf(EncoderRef(0, 0))
                val perpEncs = listOf(EncoderRef(0, 1))
                val lazyImu: LazyImu = PinpointIMU(pv)

                return DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    encoderGroups,
                    Arrays.asList(md.leftFront, md.leftBack),
                    Arrays.asList(md.rightFront, md.rightBack),
                    emptyList(),
                    emptyList(),
                    parEncs,
                    perpEncs,
                    lazyImu,
                    md.voltageSensor,
                    {
                        MotorFeedforward(
                            MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick
                        )
                    },
                    0
                )
            }
        }

        manager.register(metaForClass(AngularRampLogger::class.java), AngularRampLogger(dvf))
        manager.register(metaForClass(ForwardPushTest::class.java), ForwardPushTest(dvf))
        manager.register(metaForClass(ForwardRampLogger::class.java), ForwardRampLogger(dvf))
        manager.register(metaForClass(LateralPushTest::class.java), LateralPushTest(dvf))
        manager.register(metaForClass(LateralRampLogger::class.java), LateralRampLogger(dvf))
        manager.register(metaForClass(ManualFeedforwardTuner::class.java), ManualFeedforwardTuner(dvf))
        manager.register(metaForClass(MecanumMotorDirectionDebugger::class.java), MecanumMotorDirectionDebugger(dvf))

        manager.register(metaForClass(ManualFeedbackTuner::class.java), ManualFeedbackTuner::class.java)
        manager.register(metaForClass(SplineTest::class.java), SplineTest::class.java)
        manager.register(metaForClass(LocalizationTest::class.java), LocalizationTest::class.java)

        FtcDashboard.getInstance().withConfigRoot { configRoot ->
            for (c in listOf(
                AngularRampLogger::class.java,
                ForwardRampLogger::class.java,
                LateralRampLogger::class.java,
                ManualFeedforwardTuner::class.java,
                MecanumMotorDirectionDebugger::class.java,
                ManualFeedbackTuner::class.java
            )) {
                configRoot.putVariable(c.simpleName, ReflectionConfig.createVariableFromClass(c))
            }
        }
    }
}
