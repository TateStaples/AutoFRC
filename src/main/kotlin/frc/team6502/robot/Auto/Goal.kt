package frc.team6502.robot.Auto

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team6502.robot.RobotContainer

/**
 * A custom field object that has a position and linked command
 */
class Goal(val name: String, val pose: Pose2d, val command: Command) {
    // TODO code this

    fun execute() {
        RobotContainer.navigation.ramsete(PathPlanner.pathTo(pose)).andThen(command).schedule()
    }
}