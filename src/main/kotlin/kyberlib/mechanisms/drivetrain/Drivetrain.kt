package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds

interface Drivetrain {
    fun drive(speeds: ChassisSpeeds)
    fun debug()
}