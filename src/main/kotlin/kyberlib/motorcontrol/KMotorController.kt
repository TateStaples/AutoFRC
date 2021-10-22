package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.controller.ArmFeedforward
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import kyberlib.math.Filters.Differentiator
import kyberlib.math.units.extensions.*

typealias GearRatio = Double
typealias BrakeMode = Boolean

/**
 * Types of encoders that may be used
 */
enum class EncoderType {
    NONE, NEO_HALL, QUADRATURE
}

/**
 * Defines types of motors that can be used
 */
enum class MotorType {
    BRUSHLESS, BRUSHED
}

/**
 * Stores data about an encoder. [reversed] means the encoder reading goes + when the motor is applied - voltage.
 */
data class KEncoderConfig(val cpr: Int, val type: EncoderType, val reversed: Boolean = false)

/**
 * A more advanced motor control with feedback control.
 */
abstract class KMotorController : KBasicMotorController() {
    // ----- configs ----- //
    /**
     * The multiplier to attach to the raw velocity. *This is not the recommended way to do this.* Try using gearRatio instead
     */
    var velocityConversionFactor = 1.0
        set(value) {
            field = value
            writeMultipler(velocityConversionFactor, positionConversionFactor)
        }

    /**
     * The multiplier to attach to the raw position. *This is not the recommended way to do this.* Try using gearRatio instead
     */
    var positionConversionFactor = 1.0
        set(value) {
            field = value
            writeMultipler(velocityConversionFactor, positionConversionFactor)
        }

    /**
     * Defines the relationship between rotation and linear motion for the motor.
     */
    var radius: Length? = null

    /**
     * Adds post-encoder gearing to allow for post-geared speeds to be set.
     */
    var gearRatio: GearRatio = 1.0
        set(value) {
            field = value
            writeMultipler(gearRatio.rpm.rotationsPerSecond, gearRatio)
        }

    /**
     * Settings relevant to the motor controller's encoder.
     */
    var encoderConfig: KEncoderConfig = KEncoderConfig(0, EncoderType.NONE)
        set(value) {
            if (configureEncoder(value)) field = value
            else System.err.println("Invalid encoder configuration")
        }

    // ----- advanced tuning ----- //
    /**
     * Proportional gain of the customControl controller.
     */
    var kP: Double
        get() = PID.p
        set(value) {
            PID.p = value
            writePid(kP, kI, kD)
        }

    /**
     * Integral gain of the customControl controller.
     */
    var kI: Double
        get() = PID.i
        set(value) {
            PID.i = value
            writePid(kP, kI, kD)
        }

    /**
     * Derivative gain of the customControl controller.
     */
    var kD: Double
        get() = PID.d
        set(value) {
            PID.d = value
            writePid(kP, kI, kD)
        }

    var PID = PIDController(0.0, 0.0, 0.0)

    fun addFeedforward(feedforward: SimpleMotorFeedforward) {
        customControl = {
            val ff = feedforward.calculate(linearVelocity.metersPerSecond, linearAcceleration.metersPerSecond)
            val pid = PID!!.calculate(linearVelocityError.metersPerSecond)
            ff + pid
        }
    }
    fun addFeedforward(feedforward: ArmFeedforward) {
        customControl = {
            val ff = feedforward.calculate(position.radians, velocity.radiansPerSecond, acceleration.radiansPerSecond)
            val pid = PID!!.calculate(positionError.radians)
            ff + pid
        }
    }

    var customControl: ((it: KMotorController) -> Double)? = null

    // ----- main getter/setter methods ----- //
    var position: Angle
        get() {
            assert(encoderConfigured)
            return (rawPosition.value / gearRatio).radians
        }
        set(value) { positionSetpoint = value }

    var linearPosition: Length
        get() = rotationToLinear(position)
        set(value) { position = linearToRotation(value) }

    var velocity: AngularVelocity
        get() {
            assert(encoderConfigured)
            val vel = rawVelocity / gearRatio
            acceleration = accelerationCalculator.calculate(vel.radiansPerSecond).radiansPerSecond
            return vel
        }
        set(value) { velocitySetpoint = value }

    var linearVelocity: LinearVelocity
        get() = rotationToLinear(velocity)
        set(value) { velocity = linearToRotation(value) }

    val positionError
        get() = position - positionSetpoint
    val linearPositionError
        get() = linearPosition - linearPositionSetpoint
    val velocityError
        get() = velocity - velocitySetpoint
    val linearVelocityError
        get() = linearVelocity - linearVelocitySetpoint

    private var accelerationCalculator = Differentiator()
    var acceleration = 0.rpm
    val linearAcceleration
        get() = rotationToLinear(acceleration)

    // ----- this is where you put the advanced controls ---- //
    /**
     * Sets the angle to which the motor should go
     */
    var positionSetpoint: Angle = 0.rotations
        private set(value) {
            field = value
            if(!closedLoopConfigured) rawPosition = value
        }

    /**
     * Sets the velocity to which the motor should go
     */
    var velocitySetpoint: AngularVelocity = 0.rpm
        private set(value) {
            field = value
            if (!closedLoopConfigured) rawVelocity = value
        }

    /**
     * Sets the linear position to which the motor should go
     */
    val linearPositionSetpoint: Length
        get() = rotationToLinear(positionSetpoint)

    /**
     * Sets the linear velocity to which the motor should go
     */
    val linearVelocitySetpoint: LinearVelocity
        get() = rotationToLinear(velocitySetpoint)

    // ----- util functions -----//
    private val linearErrorMessage = "You must set the wheel radius before using linear values"
    private fun linearToRotation(len: Length): Angle {
        assert(linearConfigured) { linearErrorMessage }
        return len.toAngle(radius!!)
    }
    private fun linearToRotation(vel: LinearVelocity): AngularVelocity {
        assert(linearConfigured) { linearErrorMessage }
        return vel.toAngularVelocity(radius!!)
    }
    private fun rotationToLinear(ang: Angle): Length {
        assert(linearConfigured) { linearErrorMessage }
        return ang.toCircumference(radius!!)
    }
    private fun rotationToLinear(vel: AngularVelocity): LinearVelocity {
        assert(linearConfigured) { linearErrorMessage }
        return vel.toTangentialVelocity(radius!!)
    }
    override fun update() {  // TODO: check overriding this works
        super.update()
        if (customControl != null) voltage = customControl!!(this)
    }

    // ----- meta information ----- //
    /**
     * Does the motor controller have a rotational to linear motion conversion defined? (i.e. wheel radius)
     * Allows for linear setpoints to be used.
     */
    private val linearConfigured
        get() = radius != null

    /**
     * Does the motor controller have an encoder configured?
     * Allows for closed-loop control methods to be used
     */
    protected val encoderConfigured
        get() = (encoderConfig.type != EncoderType.NONE && encoderConfig.cpr > 0)

    /**
     * Does the motor have closed-loop gains set?
     * Allows for closed-loop control methods to be used
     */
    private val closedLoopConfigured
        get() = encoderConfigured && customControl != null

    // ----- low level getters and setters (customized to each encoder type) ----- //
    abstract fun resetPosition(position: Angle = 0.rotations)
    fun resetPosition(position: Length) {
        resetPosition(linearToRotation(position))
    }
    abstract var rawPosition: Angle
        protected set
    abstract var rawVelocity: AngularVelocity
        protected set
    abstract var currentLimit: Int

    protected abstract fun writePid(p: Double, i: Double, d: Double)

    /**
     * Set the conversion multipliers of postion and velocity
     */
    protected abstract fun writeMultipler(mv: Double, mp: Double)


    /**
     * Configures the respective ESC encoder settings when a new encoder configuration is set
     */
    protected abstract fun configureEncoder(config: KEncoderConfig): Boolean
}
