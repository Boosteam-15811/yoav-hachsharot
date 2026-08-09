package org.firstinspires.ftc.teamcode.commands

import com.seattlesolvers.solverslib.command.Command
import org.firstinspires.ftc.teamcode.alonlib.units.PercentOutput
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleConstants.EXAMPLE_POWER_LEVEL
import org.firstinspires.ftc.teamcode.subsystems.example.ExampleSubsystem

fun ExampleSubsystem.defaultExampleCommand(): Command = run { setMotorPower(0.0) }

fun ExampleSubsystem.exampleCommand(power: PercentOutput): Command = run { setMotorPower(power) }

fun ExampleSubsystem.runAtFullPowerCommand(): Command = run { setMotorPower(EXAMPLE_POWER_LEVEL) }
