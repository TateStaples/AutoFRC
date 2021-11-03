package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.SpeedController

/**
 * Wraps a WPI SpeedController to use the KBasicMotorController API
 */
class KSpeedController(private val m_speedController: SpeedController) : KBasicMotorController() {
    private companion object{
        var id = 1
    }

    val my_id = id
    init {
        id += 1

    }
    override var brakeMode: BrakeMode  // this doesn't work
        get() = false
        set(value) {m_speedController.stopMotor()}
    override var reversed: Boolean
        get() = m_speedController.inverted
        set(value) {m_speedController.inverted = value}
    override val identifier: String
        get() = "KSpeedController #$my_id"
    override var rawPercent: Double
        get() = m_speedController.get()
        set(value) {m_speedController.set(value)}

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc.followers.size == 0) kmc.notifier.startPeriodic(0.005)
        kmc.followers.add(this)
    }
}