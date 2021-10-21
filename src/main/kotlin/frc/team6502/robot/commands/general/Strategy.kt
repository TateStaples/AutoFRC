package frc.team6502.robot.commands.general

import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.balls.Shoot
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.drive.Search
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches

/**
 * Offers a high level strategy loop. Main Brain center of the robot.
 * It checks what part of the game loop the robot is in and then dispatches the next commands
 */
object Strategy {
    private var collectedBalls = 0
    private val foundBalls
        get() = Navigation.field.goals.size

    private var goalPose = Pose2d(171.532.inches, 79.inches, 0.degrees)

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

    /**
     * Paths to the goal and then dispenses
     */
    private fun shoot() {
        CommandManager.enqueue(AutoDrive(goalPose))
        CommandManager.enqueue(Shoot)
    }

    /**
     * Drives to each ball and picks it up
     */
    private fun collectBalls() {
        // TODO: implement traveling salesman optimization
        for (goal in Navigation.field.goals) {
            CommandManager.enqueue(goal.command)
        }
    }

    /**
     * Searches for more balls to pick up
     */
    private fun searchForBalls() {
        CommandManager.enqueue(Search)
    }
}