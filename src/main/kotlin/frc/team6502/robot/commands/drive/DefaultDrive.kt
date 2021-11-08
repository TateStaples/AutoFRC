package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.Robot
import frc.team6502.robot.RobotContainer

/**
 * The default drive command. Systemically takes user important and applies normal PIDF manipulations to make it happen
 * @property Drivetrain required subsystem for this command
 */
class DefaultDrive: CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    val turnFilter = SlewRateLimiter(200.0)
    val forwardFilter = SlewRateLimiter(10.0)

    /**
     * Execute part of the drive loop. Get controller values and apply the information.
     */
    override fun execute() {
        val turn = RobotContainer.controller.rightX.value
        val forward = RobotContainer.controller.leftY.value

        val speeds = ChassisSpeeds(forward, 0.0, turn)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = Constants.AUTO
}