package kyberlib.math.filters

class Integrator : Filter() {
    override var prevTime = -1.0
    private var value = 0.0
    override fun calculate(d: Double): Double {
        // right-hand approximation
        value = if (prevTime != -1.0) value * dt else 0.0
        prevTime = time
        return get()
    }
    override fun get(): Double = value
}