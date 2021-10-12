package kyberlib.motorcontrol.swerve

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState
import frc.team6502.kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.metersPerSecond

open class SwerveModule (val location: Translation2d, val driveMotor: KMotorController, val turnMotor: KMotorController) {
    // turn controls
    private var rotation
        get() = turnMotor.position.normalized
        set(value) {
            turnMotor.positionSetpoint = value  // TODO: No idea if this works
        }

    // drive info
    private var speed
        get() = driveMotor.linearVelocity
        set(value) {driveMotor.linearVelocity = value}
    private var driveVoltage
        get() = driveMotor.voltage
        set(value) {driveMotor.voltage = value}

    // general info vars
    var state
        get() = SwerveModuleState(speed.metersPerSecond, rotation)
        set(value) {
            val optimized = SwerveModuleState.optimize(value, rotation)
            rotation = optimized.angle.k
            speed = optimized.speedMetersPerSecond.metersPerSecond
        }

    val breakState
        get() = SwerveModuleState(0.0, Rotation2d(location.x, location.y).rotateBy(90.degrees))
}
