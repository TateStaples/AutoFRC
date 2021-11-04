package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import kyberlib.command.Debuggable
import kyberlib.math.invertIf

/**
 * A basic motor controller. No closed-loop control
 */
abstract class KBasicMotorController : Sendable, Debuggable() {
    protected var controlMode = ControlMode.NULL
    // ------ configs ----- //
    /**
     * Controls how the motor will stop when set to 0. If true the motor will brake instead of coast.
     */
    abstract var brakeMode: BrakeMode

    /**
     * Determines if the motor should run in the opposite direction
     */
    abstract var reversed: Boolean

    /**
     * If enabled, the motor controller will print additional information to stdout.
     */
    var debug = false

    /**
     * The prefix used by this motor for logging of errors and debug information.
     */
    abstract override var identifier: String

    /**
     * Whether the motor is connected to a real Robot
     */
    protected val real: Boolean
        get() = RobotBase.isReal()

    // ------ low-level write methods ----- //
    /**
     * Sets the voltage without changing the control mode
     */
    protected fun safeSetVoltage(voltage: Double) {
        val prevMode = controlMode
        this.voltage = voltage
        controlMode = prevMode
    }
    /**
     * What percent output is currently being applied?
     */
    var percent: Double = 0.0
        get() = if (real) rawPercent else field
        set(value) {
            controlMode = ControlMode.VOLTAGE
            if (real) rawPercent = value else field = value
        }

    /**
     * Native level get and set of motor percent.
     * Recommend using Percent because it is safer
     */
    protected abstract var rawPercent: Double

    /**
     * Sets controller voltage directly
     */
    var voltage: Double
        get() = percent * vbus
        set(value) {
            value.coerceIn(0.0 , vbus)
            percent = (value / vbus)
        }

    /**
     * The voltage available to the motor
     */
    private val vbus = if (real) RobotController.getBatteryVoltage() else 12.0

    /**
     * The notifier to update the motor continuously
     */
    internal val notifier = Notifier { update() }

    /**
     * True if this motor is following another.
     */
    var isFollower = false
        protected set

    operator fun plusAssign(kmc: KBasicMotorController) {
        kmc.follow(this)
    }

    internal val followers = arrayListOf<KBasicMotorController>()

    /**
     * Follow a motor
     */
    fun follow(kmc: KBasicMotorController) {
        isFollower = true
        followTarget(kmc)
    }

    /**
     * Native level follow
     */
    protected abstract fun followTarget(kmc: KBasicMotorController)

    /**
     * Internal update function
     */
    open fun update() {
        updateFollowers()
    }

    /**
     * Update all followers to match voltage
     */
    private fun updateFollowers() {
        for (follower in followers) {
             follower.percent = percent.invertIf { follower.reversed }
             follower.update()
        }
    }

    /**
     * Logs an error to the driver station window
     */
    fun logError(text: String) {
        DriverStation.reportError("[$identifier] $text", false)
    }

    /**
     * Logs debug information to the driver station window if debug=true
     */
    fun logDebug(text: String) {
        if (debug) println("[$identifier] $text")
    }

    /**
     * Converts motor in sendable graphics widget
     */
    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("Encoder")
        builder.addDoubleProperty("Voltage", this::voltage) { this.voltage = it }
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "Voltage" to voltage
        )
    }

    override fun toString(): String {
        return "Motor($identifier)"
    }
}
