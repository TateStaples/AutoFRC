package frc.team6502.kyberlib.math.units.extensions

import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.team6502.kyberlib.math.units.AngleConversions
import frc.team6502.kyberlib.math.units.KUnit
import frc.team6502.kyberlib.math.units.Unitless
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

typealias Angle = KRotation
//typealias Heading = KUnit<Unitless>


class KRotation(val value: Double) : Rotation2d(value) {
    val rotations
        get() = value / AngleConversions.rotationsToRadians

    fun encoderAngle(cpr: Int) = (value / AngleConversions.rotationsToRadians) * (cpr * 4)
    fun toCircumference(radius: Length) = Length(value * radius.value)
    fun subtractNearest(other: Angle): Angle {
        val diff = (value - other.value + PI) % TAU - PI
        return Angle(if (diff < -PI) diff + TAU else diff)
    }
}

const val TAU = 2 * PI

val Double.radians get() = Angle(this)
val Double.degrees get() = Angle(this * AngleConversions.degreesToRadians)
val Double.rotations get() = Angle(this * AngleConversions.rotationsToRadians)
fun Double.encoderAngle(cpr: Int) = (this / (cpr * 4.0)).rotations

val Number.radians get() = toDouble().radians
val Number.degrees get() = toDouble().degrees
val Number.rotations get() = toDouble().rotations
fun Number.encoderAngle(cpr: Int) = toDouble().encoderAngle(cpr)

//val Angle.radians get() = value
//val Angle.degrees get() = value / AngleConversions.degreesToRadians
//val Angle.rotations get() = value / AngleConversions.rotationsToRadians

//val Angle.sin get() = sin(value)
//val Angle.cos get() = cos(value)
//val Angle.tan get() = tan(value)

//fun Angle.toCircumference(radius: Length) = Length(value * radius.value)
//fun Angle.subtractNearest(other: Angle): Angle {
//    val diff = (value - other.value + PI) % TAU - PI
//    return Angle(if (diff < -PI) diff + TAU else diff)
//}
//
//fun Angle.encoderAngle(cpr: Int) = (value / AngleConversions.rotationsToRadians) * (cpr * 4)
