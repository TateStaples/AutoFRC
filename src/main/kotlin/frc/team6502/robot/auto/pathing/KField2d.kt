package frc.team6502.robot.auto.pathing

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.meters

/**
 * Updated Field class to implement new behaviours
 *
 * Should now be able to dynamically load obstacles and objectives
 * This allows the robot where it can and should go
 * @author TateStaples
 */
class KField2d : Field2d() {
    val obstacles = ArrayList<Obstacle>()
    val goals = ArrayList<Goal>()
    val width = 4.3569128.meters.value
    val height = 2.8275026.meters.value

    /**
     * Checks if a position is not obstructed
     * @param x the x coordinate of the position
     * @param y the y coordinate of the position
     * @return true/false of whether the position is free
     */
    fun inField(x: Double, y: Double): Boolean {
        return inField(Translation2d(x, y));
    }

    /**
     * Checks if a position is not obstructed
     * @param point translation representing the position to be checked
     * @return true/false of whether the position is free
     */
    fun inField(point: Translation2d): Boolean {
        return point.x in 0.0..width && point.y in 0.0..height && !hitObstacle(point)
    }

    /**
     * Checks if the point is obstructed by an obstacle
     */
    private fun hitObstacle(point: Translation2d): Boolean {
        for (obstacle in obstacles) {
            if (obstacle.contains(point)) return true
        }
        return false
    }

    private val sameDistance = 1.feet

    /**
     * Adds a goal to the field. Checks to prevent duplicates
     * @param estimatedPosition the position that the goal is locates
     * @param time the time that the observation of the position was made
     * @param name the name of the type of goal
     */
    fun addGoal(estimatedPosition: Translation2d, time: Double, name: String) {
        val similarGoals = goals.filter { it.name == name }
        if (similarGoals.any { it.position.getDistance(estimatedPosition) < sameDistance.value })  // TODO: this bad
            return
        val newGoal = Goal(name, estimatedPosition)
        goals.add(newGoal)
    }
}