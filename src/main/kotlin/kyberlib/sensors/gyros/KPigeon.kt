package kyberlib.sensors.gyros

import com.ctre.phoenix.sensors.PigeonIMU
import kyberlib.math.units.extensions.KRotation
import kyberlib.math.units.extensions.degrees

class KPigeon(port: Int) : PigeonIMU(port), KGyro {
    override var heading: KRotation = 0.degrees
        get() =
            if (real) fusedHeading.degrees
            else field
        set(value) {
            if (real) fusedHeading = value.degrees
            else field = value
        }
}