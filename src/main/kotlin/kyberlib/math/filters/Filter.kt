package kyberlib.math.filters

import edu.wpi.first.wpilibj.Timer

abstract class Filter {
    val time: Double
        get() = Timer.getFPGATimestamp()
    protected abstract var prevTime: Double
    protected val dt: Double
        get() = time - prevTime

    abstract fun calculate(d: Double): Double
    abstract fun get(): Double
}