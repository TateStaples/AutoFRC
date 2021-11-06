package frc.team6502.robot.commands

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team6502.robot.auto.Navigation
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.Pew

class Zap : InstantCommand() {
    override fun execute() {
        val p = Pew(Navigation.pose)
        Simulation.instance.include(p)
    }
}