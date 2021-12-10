package frc.team6502.robot.commands.balls

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.general.Strategy
import frc.team6502.robot.subsystems.Shooter
import kyberlib.math.units.extensions.seconds
import java.sql.Statement

/**
 * Intake new balls
 */
class Intake : CommandBase() {
    private val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        Shooter.intake.voltage = -8.0
    }

    override fun end(interrupted: Boolean) {
        Shooter.intake.voltage = 0.0
        timer.reset()
        Strategy.collectedBalls++
    }

    override fun isFinished(): Boolean {
        return Constants.AUTO && timer.hasElapsed(3.0)
    }
}