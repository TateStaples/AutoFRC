package kyberlib.motorcontrol.swerve

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.k

open class StandardSwerveModule (location: Translation2d, private val driveMotor: KMotorController, private val turnMotor: KMotorController) : SwerveModule(location) {
    // turn controls
    override var rotation: Rotation2d
        get() = turnMotor.position.normalized
        set(value) {
            turnMotor.position = value.k  // TODO: maybe add custom controls (ie Feedforward)
        }

    // drive info
    override var speed
        get() = driveMotor.linearVelocity
        set(value) {
            driveMotor.linearVelocity = value
        }


    override fun debug() {

    }
}
