package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.Debug

/**
 * The default drive command. Systemically takes user important and applies normal PIDF manipulations to make it happen
 * @property Drivetrain required subsystem for this command
 */
object DefaultDrive: CommandBase(), Debug {
    init {
        addRequirements(Drivetrain)
    }

    private val turnFilter = SlewRateLimiter(200.0)
    private val forwardFilter = SlewRateLimiter(10.0)

    private val turn
        get() = RobotContainer.controller.rightX.value
    private val forward
        get() = RobotContainer.controller.leftY.value

    /**
     * Execute part of the drive loop. Get controller values and apply the information.
     */
    override fun execute() {
        val speeds = ChassisSpeeds(forwardFilter.calculate(forward), 0.0, turnFilter.calculate(turn))
        Drivetrain.drive(speeds)
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "turn" to turn,
            "forward" to forward
        )
    }

    override fun isFinished() = Constants.AUTO
}