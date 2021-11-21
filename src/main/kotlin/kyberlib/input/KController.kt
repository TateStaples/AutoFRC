package kyberlib.input

import edu.wpi.first.wpilibj.Joystick

/**
 * Base class of any kyberlib controller.
 * Wraps the internal raw joystick
 */
abstract class KController(port: Int = 0) {
    protected val joystick = Joystick(port)
}
