package kyberlib.motorcontrol

import edu.wpi.first.wpilibj.controller.PIDController
import kyberlib.math.invertIf
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.AngularVelocity
import kyberlib.math.units.extensions.rotations
import kyberlib.math.units.extensions.rpm
import frc.team6502.kyberlib.motorcontrol.BrakeMode
import frc.team6502.kyberlib.motorcontrol.KEncoderConfig
import frc.team6502.kyberlib.motorcontrol.KMotorController

class KSimulatedESC(val name: String, apply: KSimulatedESC.() -> Unit = {}) : KMotorController() {

    private val posController = PIDController(0.0, 0.0, 0.0)
    private val velController = PIDController(0.0, 0.0, 0.0)

    var currentPos = 0.rotations
    set(value) {
        field = value
        percent = posController.calculate(currentPos.rotations, pos.rotations) + (feedForward?.asDouble?.div(12.0) ?: 0.0)
    }
    var currentVel = 0.rpm
        set(value) {
            field = value
            percent = velController.calculate(currentVel.rpm, vel.rpm) + (feedForward?.asDouble?.div(12.0) ?: 0.0)
        }
    private var pos = 0.rotations
    private var vel = 0.rpm

    private var percent = 0.0

    init {
        this.apply(apply)
    }

    override fun writePid(p: Double, i: Double, d: Double) {
        println(posController)
        posController.p = p
        posController.i = i
        posController.d = d
    }

    override fun writeMultipler(mv: Double, mp: Double) {

    }

    override fun writePosition(position: Angle) {
        pos = position
//        vel = kyberlib.math.units.extensions.rpm
//        currentVel = kyberlib.math.units.extensions.rpm
        percent = posController.calculate(currentPos.rotations, pos.rotations) + (feedForward?.asDouble?.div(12.0) ?: 0.0)
    }

    override fun writeVelocity(vel: AngularVelocity) {
        pos = 0.rotations
        currentPos = 0.rotations
        this.vel = vel
        percent = velController.calculate(currentVel.rpm, currentVel.rpm) + (feedForward?.asDouble?.div(12.0) ?: 0.0)
    }

    override fun readPosition(): Angle = pos

    override fun readVelocity(): AngularVelocity = vel

    override fun writeCurrentLimit(limit: Int) { }

    override fun zeroPosition() {
        pos = 0.rotations
    }

    override fun configureEncoder(config: KEncoderConfig) = true

    override val identifier = "sim"

    override fun followTarget(kmc: KBasicMotorController) {
        kmc.followers.add(this)
        kmc.notifier.startPeriodic(0.005)
    }

    override fun writeBrakeMode(brakeMode: BrakeMode) { }

    override fun writeReversed(reversed: Boolean) { }

    override fun writePercent(value: Double) {
        percent = value + (feedForward?.asDouble?.div(12.0) ?: 0.0)
        percent = percent.invertIf { reversed && !isFollower }
    }

    override fun readPercent() = percent
}
