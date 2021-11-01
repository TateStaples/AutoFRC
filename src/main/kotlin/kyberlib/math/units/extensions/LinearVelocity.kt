package kyberlib.math.units.extensions

import kyberlib.math.units.*

typealias LinearVelocity = KUnit<Div<Meter, Second>>

val Number.metersPerSecond get() = this.meters / 1.seconds
val Number.feetPerSecond get() = this.feet / 1.seconds
val Number.milesPerHour get() = this.miles / 1.hours

val LinearVelocity.metersPerSecond get() = value
val LinearVelocity.feetPerSecond get() = value / LengthConversions.feetToMeters
val LinearVelocity.milesPerHour get() = value / (LengthConversions.milesToFeet * LengthConversions.feetToMeters / (TimeConversions.hoursToMinutes * TimeConversions.minutesToSeconds))
fun LinearVelocity.toAngularVelocity(radius: Length) = (value / radius.value).radiansPerSecond

operator fun LinearVelocity.times(other: KUnit<Second>): KUnit<Meter> {
    val unit = KUnit<Meter>(this.metersPerSecond * other.seconds)
    unit.units = "Meters"
    return unit
}