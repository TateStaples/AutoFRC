package frc.team6502.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer

/**
 * The default drive command. Systemically takes user important and applies normal PIDF manipulations to make it happen
 * @property Drivetrain required subsystem for this command
 */
class DefaultDrive: CommandBase() {
    init {
        addRequirements(Drivetrain)
    }

    override fun initialize() {

    }

    /**
     * Execute part of the drive loop. Get controller values and apply the information.
     */
    override fun execute() {
        val turn = RobotContainer.controller.rightX
        val forward = RobotContainer.controller.leftY
        val strafe = RobotContainer.controller.leftX

        SmartDashboard.putNumber("y", forward.value)
        SmartDashboard.putNumber("x", turn.value)

        val speeds = ChassisSpeeds(forward.value / 10, strafe.value / 10, turn.value / 10)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = Constants.AUTO
}