package frc.team6502.robot.commands.balls

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Shooter

/**
 * Intake new balls
 */
object Intake : CommandBase() {
    init {
        addRequirements(Shooter)
    }

    override fun execute() {
        Shooter.intake.setVoltage(1.0)
    }

    override fun end(interrupted: Boolean) {
        Shooter.intake.setVoltage(0.0)
    }
}