package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.controller.PIDController
import kyberlib.math.invertIf
import kyberlib.math.units.extensions.*

class KSimulatedESC(val name: String) : KMotorController() {

    private val posController = PIDController(0.0, 0.0, 0.0)
    private val velController = PIDController(0.0, 0.0, 0.0)

    override var rawPosition: Angle = 0.degrees
        set(value) {
            percent = posController.calculate(field.rotations, field.rotations)
            field = value
        }

    override var rawVelocity: AngularVelocity = 0.rpm
        set(value) {
            field = value
            velController.calculate(field.rpm, field.rpm)
        }

    override var currentLimit: Int = -1

    override fun configureEncoder(config: KEncoderConfig) = true

    override val identifier = "sim"

    override var brakeMode: BrakeMode = false

    override var reversed: Boolean = false

    override var percent: Double = 0.0
        set(value) {
            field =  value.invertIf { reversed && !isFollower }
        }

    override fun writePid(p: Double, i: Double, d: Double) {
        println(posController)
        posController.p = p
        posController.i = i
        posController.d = d
    }

    override fun writeMultipler(mv: Double, mp: Double) {

    }

    override fun resetPosition(position: Angle) {
        rawPosition = position
    }

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
        kmc.notifier.startPeriodic(0.005)
    }
}
