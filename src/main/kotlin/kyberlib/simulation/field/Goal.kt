package kyberlib.simulation.field

import edu.wpi.first.wpilibj.MedianFilter
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.math.units.extensions.degrees

/**
 * A custom field object that has a position and linked command
 */
class Goal(val name: String, var position: Translation2d, val uponArrival: Command? = null) {
    private val fieldObject: FieldObject2d
        get() = KField2d.getObject(name)

    init { add() }

    private fun add() {
        val prevPoses = fieldObject.poses
        prevPoses.add(Pose2d(position, 0.degrees))
        fieldObject.poses = prevPoses
        KField2d.goals.add(this)
    }
    private fun remove() {
        val prevPoses = fieldObject.poses
        prevPoses.remove(Pose2d(position, 0.degrees))
        fieldObject.poses = prevPoses
        KField2d.goals.remove(this)
    }

    val xFilter = MedianFilter(5).apply { calculate(position.x) }
    val yFilter = MedianFilter(5).apply { calculate(position.y) }

    fun updatePosition(newPos: Translation2d) {
        position = Translation2d(xFilter.calculate(newPos.x), yFilter.calculate(newPos.y))
    }
    /**
     * Command to execute when trying to complete this gaol
     */
    val command: Command
        get() {
            val pathCommand = AutoDrive(position)
            if (uponArrival != null) return pathCommand.andThen(uponArrival).andThen(this::remove)
            return pathCommand.andThen(this::remove)
        }

}