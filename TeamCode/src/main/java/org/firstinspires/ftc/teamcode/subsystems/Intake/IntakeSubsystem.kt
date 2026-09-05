package org.firstinspires.ftc.teamcode.subsystems.Intake

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.RobotMap.Intake.EXTENSION_MOTOR_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.LEFT_ANGLE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.LEFT_INTAKE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.MOTOR_TYPE
import org.firstinspires.ftc.teamcode.RobotMap.Intake.RIGHT_ANGLE_SERVO_ID
import org.firstinspires.ftc.teamcode.RobotMap.Intake.RIGHT_INTAKE_SERVO_ID
import org.firstinspires.ftc.teamcode.alonlib.TelemetryLevel
import org.firstinspires.ftc.teamcode.alonlib.commands.SubsystemBase
import org.firstinspires.ftc.teamcode.alonlib.hardware.Data
import org.firstinspires.ftc.teamcode.alonlib.hardware.motors.HaMotor
import org.firstinspires.ftc.teamcode.alonlib.hardware.servos.HaServo
import org.firstinspires.ftc.teamcode.alonlib.math.geometry.Rotation2d
import org.firstinspires.ftc.teamcode.alonlib.units.compareTo
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.MAX_ANGLE
import org.firstinspires.ftc.teamcode.subsystems.Intake.IntakeConstants.MIN_ANGLE

class IntakeSubsystem(val hardwareMap: HardwareMap, val telemetry: Telemetry, val telemetryLevel: TelemetryLevel):
    SubsystemBase()
{
    // --- hardware declarations ---
    val intakeMotor = HaMotor(hardwareMap, EXTENSION_MOTOR_ID,MOTOR_TYPE).apply {
        runningDirection = Data.Motors.Direction.FORWARD
        zeroPowerBehavior = Data.Motors.ZeroPowerBehavior.FLOAT
        runMode = Data.Motors.RunMode.POSITION_CONTROL
    }
    val leftAngleServo = HaServo(hardwareMap, LEFT_ANGLE_SERVO_ID, Data.Servos.Mode.FULL_RANGE ,Data.Servos.Type.AxonMax, ).apply {
        minPosition = MIN_ANGLE
        maxPosition = MAX_ANGLE
    }
    val rightAngleServo = HaServo(hardwareMap, RIGHT_ANGLE_SERVO_ID, Data.Servos.Mode.FULL_RANGE ,Data.Servos.Type.AxonMax, leftAngleServo).apply {
        minPosition = MIN_ANGLE
        maxPosition = MAX_ANGLE
    }
    val rightIntakeServo = HaServo(hardwareMap, RIGHT_INTAKE_SERVO_ID, Data.Servos.Mode.CR, Data.Servos.Type.AxonMax)
    val leftIntakeServo = HaServo(hardwareMap, LEFT_INTAKE_SERVO_ID, Data.Servos.Mode.CR, Data.Servos.Type.AxonMax)

    val currentPosition : Rotation2d
        get() = rightAngleServo.position

    var currentPositionSetpoint : Rotation2d
        get() = rightAngleServo.position
        set(setpoint) {
            rightAngleServo.position = setpoint
        }

    var maxPosition : Rotation2d = MAX_ANGLE

    val isAtMaxPosition : Boolean
        get() = currentPosition >= MAX_ANGLE


    


}