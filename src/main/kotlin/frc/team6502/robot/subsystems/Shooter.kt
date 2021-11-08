package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Shooter.intake
import frc.team6502.robot.subsystems.Shooter.shooterMotor
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax

/**
 * This controls the ball pipeline.
 * @property intake motor in charge of pulling balls in
 * @property shooterMotor motor that releases balls
 */
object Shooter : SubsystemBase() {
    val intake = KSparkMax(Constants.INTAKE_ID, MotorType.BRUSHLESS).apply {
        brakeMode = true
        reversed = false
        currentLimit = 40
    }
    val shooterMotor = KSparkMax(Constants.SHOOTER_ID, MotorType.BRUSHLESS).apply {
        brakeMode = true
        reversed = false
        currentLimit = 40
    }
}