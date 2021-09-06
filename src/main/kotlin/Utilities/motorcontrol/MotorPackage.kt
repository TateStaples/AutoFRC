package frc.team6502.kyberlib.motorcontrol

import com.revrobotics.CANEncoder
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import frc.team6502.kyberlib.math.Differentiator
import frc.team6502.kyberlib.motorcontrol.KMotorController

class MotorPackage(val motor: CANSparkMax, val pid: PIDController, val ff: SimpleMotorFeedforward) {
    val acceleration = Differentiator()

    val encoder = motor.encoder

    var speed
        get() = encoder.velocity
        set(value) = drive(value)

    val position
        get() = encoder.position

    fun drive(s: Double) {
        val voltage = pid.calculate(speed, s) + ff.calculate(speed, acceleration.calculate(speed))
        motor.setVoltage(voltage)
    }

    fun zeroPosition() {
        encoder.position = 0.0
    }

}
