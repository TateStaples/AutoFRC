package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.RobotController
import frc.team6502.kyberlib.motorcontrol.BrakeMode
import kyberlib.math.invertIf

/**
 * A basic motor controller.
 */
abstract class KBasicMotorController {
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

    // ------ low-level write methods ----- //
    /**
     * What percent output is currently being applied?
     */
    abstract var percent: Double

    /**
     * Sets controller voltage directly
     */
    protected var voltage: Double = 0.0
        set(value) {
            field = value
            value.coerceAtMost(vbus)
            percent = (value / vbus)
        }

    private var vbus = if (RobotBase.isReal()) RobotController.getBatteryVoltage() else 12.0

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

    protected fun updateFollowers() {
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
}
