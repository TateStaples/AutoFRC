package kyberlib.math.units.extensions

import kyberlib.math.units.KUnit
import kyberlib.math.units.LengthConversions.feetToMeters
import kyberlib.math.units.LengthConversions.inchesToFeet
import kyberlib.math.units.LengthConversions.milesToFeet
import kyberlib.math.units.Meter
import kyberlib.math.units.Prefixes


typealias Length = KUnit<Meter>

fun Distance(value: Double): Length {
    val unit = Length(value)
    unit.units = "Meters"
    return unit
}

val Number.meters get() = Distance(this.toDouble())
val Number.centimeters get() = Distance(this.toDouble() * Prefixes.centi)
val Number.miles get() = Distance(this.toDouble() * milesToFeet * feetToMeters)
val Number.feet get() = Distance(this.toDouble() * feetToMeters)
val Number.inches get() = Distance(this.toDouble() * inchesToFeet * feetToMeters)

val Length.meters get() = value
val Length.centimeters get() = value / Prefixes.centi
val Length.miles get() = value / feetToMeters / milesToFeet
val Length.feet get() = value / feetToMeters
val Length.inches get() = value / (inchesToFeet * feetToMeters)
fun Length.toAngle(radius: Length) = (value / radius.value).radians
