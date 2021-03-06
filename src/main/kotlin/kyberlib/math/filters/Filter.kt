package kyberlib.math.filters

import kyberlib.command.Game

/**
 * Class that manipulates a incoming steam of doubles
 */
abstract class Filter {
    /**
     * The current time of the robot
     */
    protected val time: Double
        get() = Game.time.toDouble()

    /**
     * Takes a new piece of data and outputs how it affected the filter
     */
    abstract fun calculate(d: Double): Double

    /**
     * Get the current value of the filter without updating it
     */
    abstract fun get(): Double
}