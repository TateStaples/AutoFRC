package frc.team6502.robot.commands.general

import edu.wpi.first.wpilibj2.command.*
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.drive.DefaultDrive
import frc.team6502.robot.subsystems.Drivetrain
import java.util.*


/**
 * The main auto command manager.
 * Allows scheduling when to do what.
 */
object CommandManager : Command {
    private val queue = LinkedList<Command>()  // change to list by Subsystem

    var activeCommand: Command? = null
        set(value) {
            value?.initialize()
            field = value
        }

    /**
     * Run the active command.
     * Also check if the command is done.
     */
    override fun execute() {
        if (!Constants.AUTO) {
            DefaultDrive.execute()
            return
        }
        if (activeCommand == null && queue.isEmpty()) {
            Strategy.plan()
//            Drivetrain.driveAllVolts(0.0, 0.0, 0.0, 0.0)
            return
        }
        if (activeCommand == null) activeCommand = next()
        activeCommand!!.execute()
        if (activeCommand!!.isFinished) {
            activeCommand = next()
        }
    }

    // ---------- add / remove --------- //
    /**
     * Adds args to queue
     * @param commands list of commands to add to the queue. Will execute these commands one at a time
     */
    fun enqueue(vararg commands: Command) {
        queue.addAll(commands)
    }

    /**
     * Put list of commands in a specific location of the queue
     * @param trajectories list of commands you want the robot to follow
     * @param index where in the queue to insert. Defaults to the front of the list
     */
    fun queueInsert(vararg commands: Command, index: Int = 0) {
        queue.addAll(index, commands.toList())
    }


    /**
     * Get the next command in the queue
     * @param remove whether to remove from the queue when you retrieve. Defaults to true
     */
    fun next(remove: Boolean = true): Command = if (remove) queue.poll() else queue.peek()

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

    // ---------- manipulate commands --------- //
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
    fun compare(command1: Command, command2: Command): Boolean {
        if (command1.javaClass != command2.javaClass) return false
        if (command1 is RamseteCommand || command1 is MecanumControllerCommand) return false
        return true
    }

    /**
     * returns a string name of command
     */
    private fun describe(command: Command): String {
        return command.javaClass.simpleName
    }

    // ---------- general command stuff --------- //
    /**
     * This is the main command. Therefore, it never ends.
     */
    override fun isFinished(): Boolean = false

    override fun getRequirements(): MutableSet<Subsystem> {
        val set = mutableSetOf<Subsystem>(Drivetrain)
        activeCommand?.let { set.addAll(it.requirements) }
        for (command in queue)
            set.addAll(command.requirements)
        return set
    }
}

