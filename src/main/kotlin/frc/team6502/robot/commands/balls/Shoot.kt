package frc.team6502.robot.commands.balls

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Shooter
import kyberlib.math.units.extensions.seconds

/**
 * Dumps all stored balls
 */
class Shoot : CommandBase() {
    val maxTime = 3.seconds
    private val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        Shooter.shooterMotor.setVoltage(2.0)
    }

    override fun end(interrupted: Boolean) {
        Shooter.intake.setVoltage(0.0)
        timer.reset()
    }

    override fun isFinished(): Boolean {
        return Constants.AUTO && timer.get().seconds < maxTime
    }
}