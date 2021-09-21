package frc.team6502.robot.commands

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.*
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Drivetrain
import java.util.*
import kotlin.collections.ArrayList

/**
 * The main auto command manager.
 * Allows scheduling when to do what.
 */
object CommandManager : Command {
    private val queue = LinkedList<Command>()  // change to list by Subsystem

    private var activeCommand: Command? = null
        set(value) {
            value?.initialize()
            field = value
        }
    /**
     * Generate a command to follow a designated trajectory
     *
     * @param trajectory path for the robot to follow
     */
    private fun ramsete(trajectory: Trajectory): RamseteCommand {
        return RamseteCommand(
            trajectory,
            RobotContainer.navigation::pose,
            RamseteController(Constants.RAMSETE_BETA, Constants.RAMSETE_ZETA),
            Drivetrain.feedforward,
            Drivetrain.difKinematics,
            Drivetrain::difWheelSpeeds,
            Drivetrain.leftPID,
            Drivetrain.rightPID,
            // RamseteCommand passes volts to the callback
            Drivetrain::driveVolts,
            Drivetrain
        )
    }
    /**
     * Generate a command to follow a designated trajectory
     * @param trajectory path for the robot to follow
     */
    private fun mecCommand(trajectory: Trajectory): MecanumControllerCommand {
        return MecanumControllerCommand(trajectory, {RobotContainer.navigation.pose}, Drivetrain.mecKinematics,
            Drivetrain.leftPID, Drivetrain.rightPID, Drivetrain.rotationPID,
            Constants.velocity.value,
            Drivetrain::drive,
            Drivetrain
        )
    }

    /**
     * Drivetrain invariant method to get follow trajectory command
     */
    fun follow(trajectory: Trajectory): Command {
        return if (Constants.MECANUM) mecCommand(trajectory)
        else ramsete(trajectory)
    }
    /**
     * Adds args trajectories to queue
     * @param trajectories list of trajectories to add to the queue. Will follow these trajectories one at a time
     */
    fun enqueue(vararg trajectories: Trajectory) {
        for (trajectory in trajectories) {
            queue.add(follow(trajectory))
        }
    }
    /**
     * Adds args trajectories to queue
     * @param commands list of commands to add to the queue. Will execute these commands one at a time
     */
    fun enqueue(vararg commands: Command) {
        queue.addAll(commands)
    }
    /**
     * Put list of trajectories in a specific location of the queue
     * @param trajectories list of trajectories you want the robot to follow
     * @param index where in the queue to insert. Defaults to the front of the list
     */
    fun queueInsert(vararg trajectories: Trajectory, index: Int = 0) {
        val commands = arrayListOf<Command>()
        for (trajectory in trajectories) {
            commands.add(follow(trajectory))
        }
        queueInsert(*commands.toTypedArray(), index = index)
    }
    /**
     * Put list of commands in a specific location of the queue
     * @param trajectories list of commands you want the robot to follow
     * @param index where in the queue to insert. Defaults to the front of the list
     */
    fun queueInsert(vararg commands: Command, index: Int = 0) {
        for ((i, command) in commands.withIndex())
            queue.add(index+i, command)
    }
    /**
     * Get the next command in the queue
     * @param remove whether to remove from the queue when you retrieve. Defaults to true
     */
    fun next(remove: Boolean = true): Command? {return if (remove) queue.poll() else queue.peek()}
    /**
     * Ends the current command
     */
    fun terminate() {
        activeCommand?.end(true)
        activeCommand = next()
    }
    /**
     * Empty all queued commands
     */
    fun clear() { queue.clear() }
    /**
     * Creates command to run commands together
     */
    fun combine(vararg commands: Command) = ParallelCommandGroup(*commands)
    /**
     * Creates command to run commands in sequence
     */
    fun sequence(vararg commands: Command) = SequentialCommandGroup(*commands)

    /**
     * Get the list of indices that match a certain command type
     */
    fun index(commandType: Class<Command>): List<Int> {
        val indices = ArrayList<Int>()
        for ((index, command) in queue.withIndex()) {
            if (command.javaClass == commandType) indices.add(index)
        }
        return indices
    }
    /**
     * Check if two commands are the same
     * @return boolean of whether they are the same
     */
    private fun compare(command1: Command, command2: Command): Boolean {
        if (command1.javaClass != command2.javaClass) return false
        if (command1 is RamseteCommand || command1 is MecanumControllerCommand) return false
        return true
    }

    private fun describe(command: Command): String {
        return command.javaClass.simpleName
    }
    /**
     * Run the active command.
     * Also check if the command is done.
     */
    override fun execute() {
        if (activeCommand == null && queue.isEmpty()) {
            Drivetrain.driveAllVolts(0.0, 0.0, 0.0, 0.0)
            return
        }
        if (activeCommand == null) activeCommand = next()
        activeCommand!!.execute()
        if (activeCommand!!.isFinished) {
            activeCommand = next()
        }
    }

    override fun isFinished(): Boolean = false

    override fun getRequirements(): MutableSet<Subsystem> {
        val set = mutableSetOf<Subsystem>(Drivetrain)
        activeCommand?.let { set.addAll(it.requirements) }
        for (command in queue)
            set.addAll(command.requirements)
        return set
    }
}