package org.firstinspires.ftc.teamcode.commands

import alonlib.commands.Command
import alonlib.units.Percentage
import alonlib.units.percent
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_POWER_LEVEL
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.defaultExampleCommand(): Command = run { setMotorPower(0.0.percent) }

fun ExampleSubsystem.exampleCommand(power: Percentage): Command = run { setMotorPower(power) }

fun ExampleSubsystem.runAtFullPowerCommand(): Command = run { setMotorPower(EXAMPLE_POWER_LEVEL) }
