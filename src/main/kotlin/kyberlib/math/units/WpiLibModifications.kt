package kyberlib.math.units

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d

val Pose2d.string: String
    get() = "pose(${this.x}, ${this.y}, ${this.rotation.degrees}"

val Translation2d.string: String
    get() = "pos(${this.x}, ${this.y}"