package kyberlib.sensors.gyros

import edu.wpi.first.wpilibj.RobotBase
import kyberlib.math.units.extensions.KRotation
import edu.wpi.first.wpilibj.interfaces.Gyro

interface KGyro {
    val real
        get() = RobotBase.isReal()
    var heading: KRotation
}