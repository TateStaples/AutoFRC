package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kyberlib.math.units.extensions.KRotation

/**
 * Interface for all pre-made Drivetrains
 */
interface KDrivetrain {
    /**
     * Moves the chassis to the designated speed
     */
    fun drive(speeds: ChassisSpeeds)

    /**
     * Prints relevant values for debugging
     */
    fun debug()

    var pose: Pose2d
    val position
        get() = pose.translation
    var heading: KRotation
}