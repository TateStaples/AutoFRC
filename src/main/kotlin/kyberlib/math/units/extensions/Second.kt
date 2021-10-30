package kyberlib.math.units.extensions

import kyberlib.math.units.KUnit
import kyberlib.math.units.Second
import kyberlib.math.units.TimeConversions

typealias Time = KUnit<Second>
internal fun time(value: Double): Time {
    val unit = Time(value)
    unit.units = "Seconds"
    return unit
}

val Number.seconds get() = time(this.toDouble())
val Number.minutes get() = time(this.toDouble() * TimeConversions.minutesToSeconds)
val Number.hours get() = time(this.toDouble() * TimeConversions.minutesToSeconds)

val Time.seconds get() = value
val Time.minutes get() = value / TimeConversions.minutesToSeconds
val Time.hours get() = value / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds)