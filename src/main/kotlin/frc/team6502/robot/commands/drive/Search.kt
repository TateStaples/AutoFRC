package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.commands.general.Strategy
import frc.team6502.robot.subsystems.Drivetrain

class Search: CommandBase() {
    override fun execute() {
        Drivetrain.drive(ChassisSpeeds(0.0, 0.0, 0.5))
    }

    override fun isFinished(): Boolean {
        return Strategy.foundBalls > 0
    }


}