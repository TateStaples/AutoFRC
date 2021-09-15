package kyberlib.math.units

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.Length
import kyberlib.math.units.extensions.meters

val Pose2d.string: String
    get() = "pose(${this.x}, ${this.y}, ${this.rotation.degrees}"
fun Pose2d(x: Length, y:Length, rotation: Angle): Pose2d = Pose2d(x.meters, y.meters, rotation)

val Translation2d.string: String
    get() = "pos(${this.x}, ${this.y}"

fun Translation2d(x: Length, y:Length): Translation2d = Translation2d(x.meters, y.meters)