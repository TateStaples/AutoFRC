package kyberlib.mechanisms.drivetrain.swerve

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import kyberlib.math.units.extensions.LinearVelocity
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.metersPerSecond

abstract class SwerveModule(val location: Translation2d) {
    abstract var rotation: Rotation2d
    abstract var speed: LinearVelocity

    var state: SwerveModuleState
        get() = SwerveModuleState(speed.metersPerSecond, rotation)
        set(value) {
            val optimized = SwerveModuleState.optimize(value, rotation)
            stateSetpoint = optimized
            rotation = optimized.angle.k
            speed = optimized.speedMetersPerSecond.metersPerSecond
        }

    var stateSetpoint: SwerveModuleState = breakState
        private set

    val breakState: SwerveModuleState
        get() = SwerveModuleState(0.0, Rotation2d(location.x, location.y).rotateBy(90.degrees))


    abstract fun debug()
}