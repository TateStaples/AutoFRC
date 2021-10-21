package kyberlib.math.units.extensions

import kyberlib.math.units.*

typealias AngularVelocity = KUnit<Div<Unitless, Second>>

val Double.radiansPerSecond get() = AngularVelocity(this)
val Double.degreesPerSecond get() = AngularVelocity(this * AngleConversions.degreesToRadians)
val Double.rpm get() = AngularVelocity(this * AngleConversions.rotationsToRadians / TimeConversions.minutesToSeconds)
val Double.rotationsPerSecond get() = AngularVelocity(this * AngleConversions.rotationsToRadians)
fun Double.encoderVelocity(cpr: Int) = AngularVelocity((this / (cpr * 4.0)) * AngleConversions.rotationsToRadians * 10)

val Number.radiansPerSecond get() = toDouble().radiansPerSecond
val Number.degreesPerSecond get() = toDouble().degreesPerSecond
val Number.rpm get() = toDouble().rpm
val Number.rotationsPerSecond get() = toDouble().rotationsPerSecond
fun Number.encoderVelocity(cpr: Int) = toDouble().encoderVelocity(cpr)

val AngularVelocity.radiansPerSecond get() = value
val AngularVelocity.degreesPerSecond get() = value / AngleConversions.degreesToRadians
val AngularVelocity.rpm get() = value * TimeConversions.minutesToSeconds / AngleConversions.rotationsToRadians
val AngularVelocity.rotationsPerSecond get() = value / AngleConversions.rotationsToRadians
fun AngularVelocity.toTangentialVelocity(radius: Length) = LinearVelocity(value * radius.value)
fun AngularVelocity.encoderVelocity(cpr: Int) = (value / (AngleConversions.rotationsToRadians * 10)) * (cpr * 4)
