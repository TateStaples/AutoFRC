package kyberlib.math.units.extensions

import edu.wpi.first.wpilibj.geometry.Rotation2d
import kyberlib.math.units.*
import kotlin.math.PI

typealias Angle = KRotation
//typealias Heading = KUnit<Radians>


class KRotation(val value: Double) : Rotation2d(value) {
    val rotations
        get() = value / AngleConversions.rotationsToRadians
    val normalized
        get() = ((rotations - rotations.toInt()) * rotations).radians

    fun encoderAngle(cpr: Int) = (value / AngleConversions.rotationsToRadians) * (cpr * 4)
    fun toCircumference(radius: Length) = Length(value * radius.value)
    fun subtractNearest(other: Angle): Angle {
        val diff = (value - other.value + PI) % TAU - PI
        return Angle(if (diff < -PI) diff + TAU else diff)
    }
}

operator fun Rotation2d.div(other: KUnit<Second>): KUnit<Div<Radian, Second>> {
    val unit = KUnit<Div<Radian, Second>>(radians / other.value)
    unit.units = "Radians / ${other.units}"
    return unit
}

val Rotation2d.k: KRotation
    get() = KRotation(this.radians)

const val TAU = 2 * PI

val Number.radians get() = Angle(this.toDouble())
val Number.degrees get() = Angle(this.toDouble() * AngleConversions.degreesToRadians)
val Number.rotations get() = Angle(this.toDouble() * AngleConversions.rotationsToRadians)
fun Number.encoderAngle(cpr: Int) = (this.toDouble() / (cpr * 4.0)).rotations
