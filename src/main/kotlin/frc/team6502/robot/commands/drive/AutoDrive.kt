package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.math.units.extensions.degrees
import kyberlib.simulation.field.KField2d

/**
 * Automatically path and drive to a pose when called
 * @param targetPose the pose to drive to
 */
class AutoDrive(var targetPose: Pose2d) : CommandBase() {
    constructor(position: Translation2d) : this(Pose2d(position, 0.degrees)) {
        rotationInvariant = true
    }

    constructor(trajectory: Trajectory) : this(trajectory.states.last().poseMeters) {
        this.trajectory = trajectory
    }

    init {
        assert(KField2d.inField(targetPose.translation)) {"Invalid location selected. The robot tried to drive to ${targetPose.translation}, which is outside of the field"}
        addRequirements(Drivetrain)
    }
    private var rotationInvariant = false

    lateinit var trajectory: Trajectory
    private val timer = Timer()

    private val calculator = RamseteController(Constants.RAMSETE_BETA, Constants.RAMSETE_ZETA)

    override fun initialize() {
        if (!this::trajectory.isInitialized)
            trajectory = if (rotationInvariant) RobotContainer.pathfinder.pathTo(RobotContainer.navigation.pose, targetPose.translation)
                        else RobotContainer.pathfinder.pathTo(RobotContainer.navigation.pose, targetPose)
        KField2d.trajectory = trajectory
        timer.start()
    }

    override fun execute() {
        val targetSpeed = calculator.calculate(RobotContainer.navigation.pose, trajectory.sample(timer.get()))
        Drivetrain.drive(targetSpeed)
//        trajectory = PathPlanner.updateTrajectory(trajectory) - this should be necesary until moving obstabcles
    }

    override fun end(interrupted: Boolean) {
        Drivetrain.leftMaster.voltage = 0.0
        Drivetrain.rightMaster.voltage = 0.0
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(trajectory.totalTimeSeconds)
    }
}