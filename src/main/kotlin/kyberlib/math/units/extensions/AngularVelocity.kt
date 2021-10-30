package kyberlib.math.units.extensions

import kyberlib.math.units.*

typealias AngularVelocity = KUnit<Div<Radian, Second>>

val Number.radiansPerSecond get() = this.radians / 1.seconds
val Number.degreesPerSecond get() = this.degrees / 1.seconds
val Number.rpm get() = this.rotations / 1.minutes
val Number.rotationsPerSecond get() = this.rotations / 1.seconds
fun Number.encoderVelocity(cpr: Int) = ((this.toDouble() / (cpr * 4.0)) * AngleConversions.rotationsToRadians * 10).radiansPerSecond

val AngularVelocity.radiansPerSecond get() = value
val AngularVelocity.degreesPerSecond get() = value / AngleConversions.degreesToRadians
val AngularVelocity.rpm get() = value * TimeConversions.minutesToSeconds / AngleConversions.rotationsToRadians
val AngularVelocity.rotationsPerSecond get() = value / AngleConversions.rotationsToRadians
fun AngularVelocity.toTangentialVelocity(radius: Length) = LinearVelocity(value * radius.value)
fun AngularVelocity.encoderVelocity(cpr: Int) = (value / (AngleConversions.rotationsToRadians * 10)) * (cpr * 4)