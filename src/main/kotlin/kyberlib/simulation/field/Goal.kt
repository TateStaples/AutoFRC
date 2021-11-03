package kyberlib.simulation.field

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.general.CommandManager
import kyberlib.math.units.extensions.degrees

/**
 * A custom field object that has a position and linked command
 */
class Goal(val name: String, val position: Translation2d, val uponArrival: Command? = null) {
    /**
     * Command to execute when trying to complete this gaol
     */
    val command: Command
        get() {
            val pathCommand = AutoDrive(Pose2d(position, 0.degrees))
            if (uponArrival != null)
                return CommandManager.sequence(pathCommand, uponArrival)
            return pathCommand
        }

}