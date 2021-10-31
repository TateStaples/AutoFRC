package kyberlib.input

import kotlin.math.abs

/**
 * Wrapper for a DoubleSuppler (that retrieves raw axis value) and applies some hyper-parameters to increase usability
 */
class KAxis(val raw: () -> Double) {

    var rate = 1.0
    var expo = 0.0
    var superRate = 0.0
    var deadband = 0.01  // how much of the controller should default to 0

    /**
     * Fancy non-linear value of the axis
     */
    val value: Double
        /**
         * Applies fancy math to make joystick non-linear
         */
        get() {
            var command = raw.invoke()

            // apply deadband
            command = when {
                command > deadband -> (1 / (1 - deadband)) * command - (deadband / (1 - deadband))
                command < -deadband -> (1 / (1 - deadband)) * command + (deadband / (1 - deadband))
                else -> 0.0
            }

            var retval = ((1 + 0.01 * expo * (command * command - 1.0)) * command)
            retval = (retval * (rate + (abs(retval) * rate * superRate * 0.01)))
            return retval
        }
}
