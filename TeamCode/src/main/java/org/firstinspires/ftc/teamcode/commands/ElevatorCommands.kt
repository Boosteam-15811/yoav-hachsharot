package org.firstinspires.ftc.teamcode.commands

import alonlib.commands.Command
import alonlib.math.geometry.AngularPositon
import org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorConstants.DEFAULT_SETPOINT
import org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorConstants.HIGH_BASKET_SETPOINT
import org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorConstants.LOW_BASKET_SETPOINT
import org.firstinspires.ftc.teamcode.subsystems.Elevator.ElevatorSubsystem

fun ElevatorSubsystem.GoTOPositionCommand(position: AngularPositon): Command = run { positionSetpoint = position }
fun ElevatorSubsystem.DefaultCommand(): Command = run { positionSetpoint = DEFAULT_SETPOINT }
fun ElevatorSubsystem.GoToHighBasketCommand(): Command = run { positionSetpoint = HIGH_BASKET_SETPOINT }
fun ElevatorSubsystem.GoToLowBasketCommand(): Command = run { positionSetpoint = LOW_BASKET_SETPOINT }
fun ElevatorSubsystem.HomingCommand(): Command = run { homing() }
