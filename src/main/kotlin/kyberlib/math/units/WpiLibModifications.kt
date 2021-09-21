package kyberlib.math.units

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kyberlib.math.units.extensions.*
import kotlin.math.sqrt

val Pose2d.string: String
    get() = "pose(${this.x}, ${this.y}, ${this.rotation.degrees}"
fun Pose2d(x: Length, y:Length, rotation: Angle): Pose2d = Pose2d(x.meters, y.meters, rotation)

val zeroPose = Pose2d(0.0, 0.0, 0.degrees)

val Translation2d.string: String
    get() = "pos(${this.x}, ${this.y}"

fun Translation2d(x: Length, y:Length): Translation2d = Translation2d(x.meters, y.meters)
fun Translation2d(x: Length, rotation: Rotation2d): Translation2d = Translation2d(x.meters, rotation)
val zeroTranslation = Translation2d(0.0, 0.0)

val ChassisSpeeds.speed: LinearVelocity
        get() = sqrt(vxMetersPerSecond * vxMetersPerSecond + vyMetersPerSecond * vyMetersPerSecond).metersPerSecond