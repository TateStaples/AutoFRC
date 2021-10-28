package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kyberlib.simulation.Simulatable

interface Drivetrain {
    fun drive(speeds: ChassisSpeeds)
    fun debug()
}