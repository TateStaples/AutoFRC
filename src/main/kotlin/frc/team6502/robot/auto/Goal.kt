package frc.team6502.robot.auto

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team6502.robot.auto.pathing.PathPlanner

/**
 * A custom field object that has a position and linked command
 */
class Goal(name: String, val position: Translation2d, val command: Command) : SequentialCommandGroup() {
    init {
        addCommands(command, CommandManager.follow(pathTo))
        this.name = name
    }
    private val pathTo
        get() = PathPlanner.pathTo(position)
}