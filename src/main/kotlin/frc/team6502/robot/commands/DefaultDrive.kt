package frc.team6502.robot.commands

import edu.wpi.first.wpilibj.SlewRateLimiter
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import kyberlib.math.units.extensions.feetPerSecond
import kyberlib.math.units.extensions.radiansPerSecond

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
    val strafeFilter = SlewRateLimiter(10.0)

    override fun initialize() {

    }

    /**
     * Execute part of the drive loop. Get controller values and apply the information.
     */
    override fun execute() {
        val turn = RobotContainer.controller.rightX.value.radiansPerSecond
        val forward = RobotContainer.controller.leftY.value.feetPerSecond
        val strafe = RobotContainer.controller.leftX.value.feetPerSecond

        SmartDashboard.putNumber("y", forward.value)
        SmartDashboard.putNumber("x", turn.value)

        val speeds = ChassisSpeeds(forward.value / 10, strafe.value / 10, turn.value / 10)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = Constants.AUTO
}