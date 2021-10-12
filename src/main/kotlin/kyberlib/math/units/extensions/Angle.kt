package kyberlib.math.units.extensions

import edu.wpi.first.wpilibj.geometry.Rotation2d
import kyberlib.math.units.AngleConversions
import kotlin.math.PI

typealias Angle = KRotation
//typealias Heading = KUnit<Unitless>


class KRotation(val value: Double) : Rotation2d(value) {
    val rotations
        get() = value / AngleConversions.rotationsToRadians
    val normalized = ((rotations - rotations.toInt()) * rotations).radians // TODO check if this works at all

    fun encoderAngle(cpr: Int) = (value / AngleConversions.rotationsToRadians) * (cpr * 4)
    fun toCircumference(radius: Length) = Length(value * radius.value)
    fun subtractNearest(other: Angle): Angle {
        val diff = (value - other.value + PI) % TAU - PI
        return Angle(if (diff < -PI) diff + TAU else diff)
    }
}

val Rotation2d.k: KRotation
    get() = KRotation(this.radians)

const val TAU = 2 * PI

val Double.radians get() = Angle(this)
val Double.degrees get() = Angle(this * AngleConversions.degreesToRadians)
val Double.rotations get() = Angle(this * AngleConversions.rotationsToRadians)
fun Double.encoderAngle(cpr: Int) = (this / (cpr * 4.0)).rotations

val Number.radians get() = toDouble().radians
val Number.degrees get() = toDouble().degrees
val Number.rotations get() = toDouble().rotations
fun Number.encoderAngle(cpr: Int) = toDouble().encoderAngle(cpr)
