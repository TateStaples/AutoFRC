package kyberlib.math

import java.math.BigDecimal
import java.math.RoundingMode
import kotlin.math.absoluteValue
import kotlin.math.roundToInt

val Double.rtf: Double get() = (this * 10000).roundToInt() / 10000.0

// conditional inversion
fun Double.invertIf(condition: () -> Boolean) = this * if (condition()) -1.0 else 1.0

fun Int.invertIf(condition: () -> Boolean) = this * if (condition()) -1 else 1

// check if very close
infix fun Double.epsilonEquals(value: Double): Boolean {
    return (this - value).absoluteValue < 0.00001
}

/**
 * Round to a given number of decimal places
 * @param decimals number of decimals to keep
 * @return rounded value
 */
fun Double.round(decimals: Int): Double {
    return BigDecimal(this).setScale(decimals, RoundingMode.HALF_EVEN).toDouble()
}