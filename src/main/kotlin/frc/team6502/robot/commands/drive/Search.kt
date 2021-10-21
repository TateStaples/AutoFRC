package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.subsystems.Drivetrain

/**
 * Shitty method to look for new balls.
 * Just spins in a circle
 */
object Search : CommandBase() {
    private var lastDetectedBallTime = -1.0
    private val time
        get() = Timer.getFPGATimestamp()

    private const val maxWait = 10.0
    private const val maxBalls = 6

    private var previousFoundBalls = 0
    private val foundBalls
        get() = Navigation.field.goals.size

    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {
        lastDetectedBallTime = -1.0
        previousFoundBalls = foundBalls
    }

    override fun execute() {
        Drivetrain.drive(ChassisSpeeds(0.0, 0.0, 0.5))
        if (previousFoundBalls != foundBalls) {
            previousFoundBalls = foundBalls
            lastDetectedBallTime = time
        }
    }

    override fun isFinished(): Boolean {
        val doneWaiting = lastDetectedBallTime != -1.0 && time - lastDetectedBallTime > maxWait
        return doneWaiting || foundBalls >= maxBalls
    }
}