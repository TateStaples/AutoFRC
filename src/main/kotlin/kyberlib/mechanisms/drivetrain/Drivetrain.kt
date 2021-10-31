package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kyberlib.simulation.Simulatable

/**
 * Interface for all pre-made Drivetrains
 */
interface Drivetrain {
    /**
     * Moves the chassis to the designated speed
     */
    fun drive(speeds: ChassisSpeeds)

    /**
     * Prints relevant values for debugging
     */
    fun debug()
}