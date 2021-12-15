package frc.team6502.robot.commands.general

import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.balls.Shoot
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.drive.Search
import kyberlib.auto.pathing.TravelingSalesman
import kyberlib.command.CommandManager
import kyberlib.command.Debug
import kyberlib.command.DebugLevel
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.zeroPose
import kyberlib.simulation.field.KField2d

/**
 * Offers a high level strategy loop. Main Brain center of the robot.
 * It checks what part of the game loop the robot is in and then dispatches the next commands
 */
object Strategy : Debug {
    var collectedBalls = 0
    val foundBalls
        get() = KField2d.goals.filter { it.name == "ball" }.size

    private var goalPose = Pose2d(0.inches, 0.inches, 0.degrees)//Pose2d(151.532.inches, 79.inches, 0.degrees)  // was 171.532
    /**
     * When all the queued commands are done, it requests here what to do next
     */
    fun plan() {
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
        log("Shooting")
        CommandManager.enqueue(AutoDrive(goalPose).andThen(Shoot()))
    }

    /**
     * Drives to each ball and picks it up
     */
    private fun collectBalls() {
        log("collecting route: $foundBalls")
        val goals = KField2d.goals.filter { it.name == "ball" }
        val points = goals.map { it.position }
        val route = TravelingSalesman(points.toMutableList(),
            RobotContainer.navigation.position, goalPose.translation
        ).bruteForce()
        log(route.toString())
        debugDashboard()
        for (waypoint in route.slice(IntRange(1, route.size-2))) {
            val goal = goals.find { it.position == waypoint }
            if (goal != null) {
                val command = goal.command
                CommandManager.enqueue(command)
            }
        }
    }

    /**
     * Searches for more balls to pick up
     */
    private fun searchForBalls() {
        log("searching")
        CommandManager.enqueue(Search())
    }

    override var priority: DebugLevel
        get() = DebugLevel.NORMAL
        set(value) {}

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "found" to foundBalls,
            "collected" to collectedBalls
        )
    }
}