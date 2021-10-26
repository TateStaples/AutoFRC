package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import kyberlib.math.invertIf

/**
 * A basic motor controller.
 */
abstract class KBasicMotorController : Sendable {
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
    abstract val identifier: String

    protected val real: Boolean
        get() = RobotBase.isReal()

    // ------ low-level write methods ----- //
    /**
     * What percent output is currently being applied?
     */
    var percent: Double = 0.0
        get() = if (real) rawPercent else field
        set(value) {
            if (real) rawPercent = value else field = value
        }

    protected abstract var rawPercent: Double

    /**
     * Sets controller voltage directly
     */
    var voltage: Double
        get() = percent * vbus
        set(value) {
            value.coerceAtMost(vbus)
            percent = (value / vbus)
        }

    protected var vbus = if (real) RobotController.getBatteryVoltage() else 12.0

    val notifier = Notifier { update() }

    init {
        notifier.startPeriodic(0.02)
    }
    /**
     * True if this motor is following another.
     */
    var isFollower = false
        protected set

    operator fun plusAssign(kmc: KBasicMotorController) {
        kmc.follow(this)
    }

    internal val followers = arrayListOf<KBasicMotorController>()
    fun follow(kmc: KBasicMotorController) {
        isFollower = true
        followTarget(kmc)
    }

    protected abstract fun followTarget(kmc: KBasicMotorController)

    /**
     * Internal update function
     */
    open fun update() {
        updateFollowers()
    }

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

    override fun initSendable(builder: SendableBuilder) {
        builder.setSmartDashboardType("Encoder")
        builder.addDoubleProperty("Voltage", this::voltage, null)
    }

}
