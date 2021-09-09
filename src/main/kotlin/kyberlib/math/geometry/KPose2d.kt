package kyberlib.math.geometry

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import kyberlib.math.units.extensions.Angle
import kyberlib.math.units.extensions.Distance
import kyberlib.math.units.extensions.meters

data class KPose2d(val x: Distance, val y: Distance, val rotation: Angle) {
    val wpiPose2d = Pose2d(x.meters, y.meters, Rotation2d.fromDegrees(rotation.degrees))
}
