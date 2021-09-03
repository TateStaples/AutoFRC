package frc.team6502.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.subsystems.Drivetrain

class AutoDrive : CommandBase() {
    init {
        addRequirements(Drivetrain)
    }
}