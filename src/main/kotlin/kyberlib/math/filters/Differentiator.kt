package kyberlib.math.filters

/**
 * Gets the rate of change of a stream of values
 */
class Differentiator : Filter() {
    override var prevTime = 0.0
    private var lastValue: Double? = null
    private var value = 0.0

    /**
     * Return the rate of change of value in units per second
     */
    override fun calculate(d: Double): Double {
        value = if (lastValue != null) (value - lastValue!!) / dt else 0.0
        lastValue = value
        prevTime = time
        return value
    }

    override fun get(): Double = value
}
