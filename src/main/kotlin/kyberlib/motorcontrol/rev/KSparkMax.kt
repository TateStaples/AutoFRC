package kyberlib.motorcontrol.rev

import com.revrobotics.CANEncoder
import com.revrobotics.CANPIDController
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.ControlType
import frc.team6502.kyberlib.motorcontrol.BrakeMode
import frc.team6502.kyberlib.motorcontrol.EncoderType
import frc.team6502.kyberlib.motorcontrol.KEncoderConfig
import frc.team6502.kyberlib.motorcontrol.KMotorController
import frc.team6502.kyberlib.motorcontrol.MotorType.BRUSHED
import frc.team6502.kyberlib.motorcontrol.MotorType.BRUSHLESS
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.AngularVelocity
import kyberlib.math.units.extensions.rotations
import kyberlib.math.units.extensions.rpm
import kyberlib.motorcontrol.CANId
import kyberlib.motorcontrol.CANRegistry
import kyberlib.motorcontrol.KBasicMotorController


/**
 * Represents a REV Robotics Spark MAX motor controller.
 * [canId] is the controller's ID on the CAN bus
 * [motorType] is the type of motor being driven. WARNING: If set incorrectly this can seriously damage hardware. You've been warned.
 * [apply] is where motor setup can occur
 */
class KSparkMax(val canId: CANId, val motorType: frc.team6502.kyberlib.motorcontrol.MotorType
                            ) : KMotorController() {

    // ----- low-level stuff ----- //
    override val identifier: String = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"

    private val _spark = CANSparkMax(canId, when (motorType) {
        BRUSHLESS -> MotorType.kBrushless
        BRUSHED -> MotorType.kBrushed
    })
    private var _enc: CANEncoder? = null
    private val _pid = _spark.pidController

    init {
        _spark.restoreFactoryDefaults()
        // running NEO with integrated encoder
        if (motorType == BRUSHLESS) {
            encoderConfig = KEncoderConfig(42, EncoderType.NEO_HALL)
        }
    }

    override var brakeMode
        get() = _spark.idleMode == CANSparkMax.IdleMode.kBrake
        set(value) {
            _spark.idleMode = if(value) CANSparkMax.IdleMode.kBrake else CANSparkMax.IdleMode.kCoast
        }

    override var percent
        get() = _spark.appliedOutput
        set(value) {_spark.set(value)}

    override var reversed: Boolean
        get() = _spark.inverted
        set(value) {_spark.inverted = value}

    override var rawVelocity: AngularVelocity
        get() = _enc!!.velocity.rpm
        set(value) {
            _pid.setReference(value.rpm, ControlType.kVelocity, 0, 0.0, CANPIDController.ArbFFUnits.kVoltage)
        }

    override var rawPosition: Angle
        get() = _enc!!.position.rotations
        set(value) {
            _pid.setReference(value.rotations, ControlType.kPosition, 0, 0.0, CANPIDController.ArbFFUnits.kVoltage)
        }

    override var currentLimit: Int = -1
        set(value) {
            _spark.setSmartCurrentLimit(value)
            field = value
        }


    override fun configureEncoder(config: KEncoderConfig): Boolean {
        return when {
            config.type == EncoderType.NEO_HALL && motorType == BRUSHLESS -> {
                _enc = _spark.encoder
                true
            }
            config.type == EncoderType.QUADRATURE && motorType == BRUSHED -> {
                _enc = _spark.getEncoder(com.revrobotics.EncoderType.kQuadrature, config.cpr)
                _enc?.inverted = config.reversed
                true
            }
            else -> {
                false
            }
        }
    }

    override fun writePid(p: Double, i: Double, d: Double) {
        _pid.p = p
        _pid.i = i
        _pid.d = d
    }

    override fun writeMultipler(mv: Double, mp: Double) {
        _enc?.velocityConversionFactor = mv
        _enc?.positionConversionFactor = mp
    }

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc is KSparkMax) {
            _spark.follow(kmc._spark, reversed)
        } else {
            kmc.followers.add(this)
            kmc.notifier.startPeriodic(0.005)
        }
    }

    override fun zeroPosition() {
        if (!encoderConfigured) {
            return logError("Cannot reset encoder position without configured encoder")
        }
        _enc?.position = 0.0
    }
}
