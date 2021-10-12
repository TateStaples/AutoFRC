package kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees

class KPigeon(port: Int) : PigeonIMU(port), KGyro {
    override val heading: KRotation
        get() = fusedHeading.degrees
}