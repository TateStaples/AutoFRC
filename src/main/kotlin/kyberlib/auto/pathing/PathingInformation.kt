package kyberlib.auto.pathing

import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

// todo: document
internal class PathingInformation(val startPosition: Translation2d, val endPosition: Translation2d){
    val center = startPosition.plus(endPosition).div(2.0)  // average
    val dis = startPosition.getDistance(endPosition)
    val shifted = endPosition.minus(startPosition)
    val rotation = Rotation2d(shifted.x, shifted.y)
    var currentPathLength = -1.0
    val width: Double
        get() = currentPathLength
    val height: Double
        get() = sqrt(currentPathLength * currentPathLength - dis * dis)
    val pathFound: Boolean
        get() = currentPathLength > 0.0

    fun update(currentPathLength: Double) {
        this.currentPathLength = currentPathLength
    }

    fun get(rho: Double, theta: Double): Translation2d {
        assert(pathFound) {"You should not sample from information until pathLength is set"}
        val x = cos(theta) * width/2 * rho
        val y = sin(theta) * height/2 * rho
        val rotated = Translation2d(x, y).rotateBy(rotation)
        return rotated.plus(center)
    }

    fun debug() {
        println("start: $startPosition, end: $endPosition, w: $width, h: $height center: $center, dis: $dis, rotation: $rotation")
    }
}