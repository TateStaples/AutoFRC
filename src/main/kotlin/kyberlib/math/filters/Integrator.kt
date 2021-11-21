package kyberlib.math.filters

/**
 * Integrates a stream of data by time
 */
class Integrator : Filter() {
    private var prevTime = 0.0
    private val dt: Double
        get() = time - prevTime
    private var value = 0.0

    /**
     * Update the filter and get the new output
     */
    override fun calculate(d: Double): Double {
        // right-hand approximation
        value = if (prevTime != -1.0) value + d * dt else 0.0
        prevTime = time
        return get()
    }

    /**
     * Get the last value without updating
     */
    override fun get(): Double = value
}