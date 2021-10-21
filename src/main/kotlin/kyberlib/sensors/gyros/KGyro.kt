package kyberlib.sensors.gyros

import kyberlib.math.units.extensions.KRotation
import edu.wpi.first.wpilibj.interfaces.Gyro

interface KGyro {
    var heading: KRotation
}