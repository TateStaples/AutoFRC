package frc.team6502.robot.commands

import frc.team6502.robot.RobotContainer
import frc.team6502.robot.auto.pathing.PathPlanner
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.feet

object Strategy {
    private var collectedBalls = 0
    private val foundBalls
        get() = RobotContainer.navigation.field.goals.size

    private var goalPose = Pose2d(10.feet, 10.feet, 0.degrees)  // TODO: get this better

    /**
     * When all the queued commands are done, it requests here what to do next
     */
    internal fun plan() {
        if (collectedBalls > 0) shoot()
        else if (foundBalls > 0) collectBalls()
        else searchForBalls()
        debug()
    }

    /**
     * Spot to debug imporatnt values
     */
    private fun debug() {}

    // TODO: implement and document these
    /**
     * Paths to the goal and then dispenses
     */
    private fun shoot() {
        val toGoal = PathPlanner.pathTo(goalPose.translation)
        CommandManager.enqueue({ CommandManager.follow(toGoal) })
        CommandManager.enqueue(Shoot)
    }

    /**
     * Drives to each ball and picks it up
     */
    private fun collectBalls() {
        // TODO: implement better navigation
        for (goal in RobotContainer.navigation.field.goals) {
            CommandManager.enqueue({goal.command})  // TODO: find a way to manipulate trajectory start position
        }
    }

    /**
     * Searches for more balls to pick up
     */
    private fun searchForBalls() {
        CommandManager.enqueue(Search)
    }
}