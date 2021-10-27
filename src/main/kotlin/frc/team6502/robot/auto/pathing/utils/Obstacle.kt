package frc.team6502.robot.auto.pathing.utils

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d

/**
 * A rectangular obstacle on the field
 *
 * @param pose the position and orientation of the obstacle
 * @param width horizontal distance from the center to the side (meters)
 * @param height vertical distance from center to top/bottom (meters)
 */
class Obstacle(val pose: Pose2d, val width: Double, val height: Double) {
    val x: Double
        get() = pose.x
    val y: Double
        get() = pose.y
    val rotation: Rotation2d
        get() = pose.rotation
    val position: Translation2d
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