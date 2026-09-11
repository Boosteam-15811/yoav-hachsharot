package org.firstinspires.ftc.teamcode

import alonlib.hardware.Data


object RobotMap {

    /** here you set the hardware ids and motor types for your subsystems
     * each subsystem has her own object with its devices and their ids or types
     * an id declaration looks like this:
     *  const val DEVICE_NAME_ID = "device name"
     *
     * a type declaration looks like this:
     *
     *  val MOTOR_NAME_TYPE = Motor.GoBILDA.type
     */

    object Example {
        const val EXAMPLE_MOTOR_ID = "example motor" // port 0 on the control hub
        val EXAMPLE_MOTOR_TYPE = Data.Motors.GoBILDA.RPM_435
    }

    object Drive {
        const val PINPOINT_ID = "pinpoint"
        const val FRONT_LEFT_MOTOR_ID = "front left motor" // port 0 on the control hub
        const val FRONT_RIGHT_MOTOR_ID = "front right motor" // port 1 on the control hub
        const val BACK_LEFT_MOTOR_ID = "back left motor" // port 2 on the control hub
        const val BACK_RIGHT_MOTOR_ID = "back right motor" // port 3 on the control hub

        val DRIVE_MOTOR_TYPE = Data.Motors.GoBILDA.RPM_435
    }

    object Elevator{
        const val LEFT_MOTOR_ID = "left elevator motor"
        const val RIGHT_MOTOR_ID = "right elevator motor"
        val MOTOR_TYPE = Data.Motors.GoBILDA.RPM_435
    }

    object Intake{
        const val EXTENSION_MOTOR_ID = "intake motor"
        const val LEFT_ANGLE_SERVO_ID = "left angle servo"
        const val RIGHT_ANGLE_SERVO_ID = "right angle servo"
        const val LEFT_INTAKE_SERVO_ID = "left intake servo"
        const val RIGHT_INTAKE_SERVO_ID = "right intake servo"
        val MOTOR_TYPE = Data.Motors.GoBILDA.RPM_84
    }

    // Add one object per subsystem here as you build out the robot, e.g.:
    //
    // object Intake {
    //     const val INTAKE_MOTOR_ID = "intake motor" // port 1 on the control hub
    //     val INTAKE_MOTOR_TYPE = Motor.GoBILDA.RPM_1150
    // }
}
