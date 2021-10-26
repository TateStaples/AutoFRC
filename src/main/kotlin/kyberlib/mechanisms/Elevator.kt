package kyberlib.mechanisms

import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.Length
import kyberlib.math.units.extensions.feet


class Elevator(vararg val motors: KMotorController, radius: Length, initialPosition: Length = 0.feet) : SubsystemBase() {
    private val master = motors[0].apply {
        this.radius = radius
        resetPosition(initialPosition)
    }
    init {
        for (info in motors.withIndex()) {
            if (info.index > 0) info.value.follow(master)
        }
    }

    var position: Length
        get() = master.linearPosition
        set(value) {
            master.linearPosition = value
        }

}