package frc.team6502.robot.commands.balls

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Shooter

/**
 * Dumps all stored balls
 */
object Shoot : CommandBase() {
    init {
        addRequirements(Shooter)
    }

    override fun execute() {
        Shooter.intake.setVoltage(-5.0)
        Shooter.shooterMotor.setVoltage(1.0)
    }

    override fun end(interrupted: Boolean) {
        Shooter.intake.setVoltage(0.0)
        Shooter.shooterMotor.setVoltage(0.0)
    }
}