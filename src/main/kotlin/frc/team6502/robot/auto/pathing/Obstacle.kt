package frc.team6502.robot.auto.pathing

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d

/**
 * A rectangular obstacle on the field
 *
 * @param pose the position and orientation of the obstacle
 * @param width horizontal distance from the center to the side (meters)
 * @param height vertical distance from center to top/bottom (meters)
 */
class Obstacle(val pose: Pose2d, val width: Double, val height: Double) {
    // TODO: maybe inflate obstacle sizes so robot doesn't hit anything
    val x
        get() = pose.x
    val y
        get() = pose.y
    val rotation
        get() = pose.rotation
    val position
        get() = pose.translation

    /**
     * Checks if a point falls within the obstacles hitbox
     */
    fun contains(point: Translation2d): Boolean {
        val centered = point.minus(position)
        val rotated = centered.rotateBy(-rotation)
        return rotated.x in -width..width && rotated.y in -height..height
    }
}