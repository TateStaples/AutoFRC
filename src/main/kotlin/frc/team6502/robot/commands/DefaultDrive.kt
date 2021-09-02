package frc.team6502.robot.commands

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.RobotContainer

class DefaultDrive: CommandBase() {


    init {
        addRequirements(Drivetrain)
    }
    override fun initialize() {

    }

    override fun execute() {
        val turn = RobotContainer.controller.rightX
        val forward = RobotContainer.controller.leftY

        SmartDashboard.putNumber("y", forward.value)
        SmartDashboard.putNumber("x", turn.value)

        val speeds = ChassisSpeeds(0.0, forward.value, turn.value)
        Drivetrain.drive(speeds)
    }

    override fun isFinished() = false
}