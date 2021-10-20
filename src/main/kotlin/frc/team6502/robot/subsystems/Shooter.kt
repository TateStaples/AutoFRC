package frc.team6502.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants

/**
 * This controls the ball pipeline.
 * @property intake motor in charge of pulling balls in
 * @property shooterMotor motor that releases balls
 */
object Shooter : SubsystemBase() {
    val intake = CANSparkMax(Constants.INTAKE_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
    }
    val shooterMotor = CANSparkMax(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
    }
}