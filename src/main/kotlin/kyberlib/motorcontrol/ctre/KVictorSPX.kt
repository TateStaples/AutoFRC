package kyberlib.motorcontrol.ctre

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix.motorcontrol.can.VictorSPX
import kyberlib.motorcontrol.*

class KVictorSPX(val canId: CANId) : KBasicMotorController() {
    private val _victor = VictorSPX(canId)

    override val identifier = CANRegistry.filterValues { it == canId }.keys.firstOrNull() ?: "can$canId"


    override var brakeMode: BrakeMode = false
        set(value) {
            field = value
            val mode = if(value) NeutralMode.Brake else NeutralMode.Coast
            _victor.setNeutralMode(mode)
        }

    override var reversed: Boolean
        get() = _victor.inverted
        set(value) {_victor.inverted = value}

    override var rawPercent: Double
        get() = _victor.motorOutputPercent
        set(value) {_victor.set(ControlMode.PercentOutput, value)}

    override fun followTarget(kmc: KBasicMotorController) {
        if (kmc is KVictorSPX) {
            _victor.follow(kmc._victor)
        } else {
            if (kmc.followers.size == 0) kmc.notifier.startPeriodic(0.005)
            kmc.followers.add(this)
        }
    }
}
