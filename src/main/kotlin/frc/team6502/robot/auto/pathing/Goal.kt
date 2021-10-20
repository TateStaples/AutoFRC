package frc.team6502.robot.auto.pathing

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team6502.robot.auto.pathing.PathPlanner
import frc.team6502.robot.commands.CommandManager

/**
 * A custom field object that has a position and linked command
 */
class Goal(val name: String, val position: Translation2d, val uponArrival: Command? = null) {
    val command: Command
        get() {
            val path = PathPlanner.pathTo(position)
            val pathCommand = CommandManager.follow(path)
            if (uponArrival != null)
                return CommandManager.sequence(pathCommand, uponArrival)
            return pathCommand
        }

}