package frc.team6502.robot.commands.balls

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.general.Strategy
import frc.team6502.robot.subsystems.Shooter
import kyberlib.math.units.extensions.seconds
import kyberlib.simulation.field.KField2d

/**
 * Dumps all stored balls
 */
class Shoot : CommandBase() {
    private val maxTime = 3.seconds
    private val timer = Timer()

    init {
        addRequirements(Shooter)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        Shooter.shooterMotor.voltage = 2.0
    }

    override fun end(interrupted: Boolean) {
        Shooter.shooterMotor.voltage = 0.0
        timer.reset()
        Strategy.collectedBalls = 0
        KField2d.goals.clear()
    }

    override fun isFinished(): Boolean {
        return Constants.AUTO && timer.hasElapsed(2.0)
    }
}