package frc.team6502.robot.Auto

import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpiutil.math.MatBuilder
import edu.wpi.first.wpiutil.math.numbers.*
import frc.team6502.kyberlib.math.units.extensions.feet
import frc.team6502.kyberlib.math.units.extensions.meters
import frc.team6502.kyberlib.math.units.extensions.metersPerSecond
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Drivetrain
import java.util.*

// TODO: put characterize stuff from last year on github
/**
 * A Subsystem to manage and update the Robots position
 * @param initialPose the pose of the robot when Navigation begins
 * @author TateStaples
 */
class Navigation(initialPose: Pose2d) : SubsystemBase() {
    val field = KField2d()  // TODO extend this class and add obstacles and goals
    val fieldTraj = field.getObject("traj")
    private val trajectoryQueue: Queue<Trajectory> = LinkedList() // should I make this into seperate class

    /**
     * A probability calculator to guess where the robot is from odometer and vision updates
     */
    private var estimator = DifferentialDrivePoseEstimator(  // TODO: think of going back to mecanum
        heading, initialPose,
        // State measurement standard deviations. X, Y, theta, dist_l, dist_r. (dist is encoder distance I think)
        MatBuilder(N5.instance, N1.instance).fill(
            0.02,
            0.02,
            0.01,
            0.02,
            0.02
        ),
        // Local measurement standard deviations. Left encoder, right encoder, gyro.
        MatBuilder(N3.instance, N1.instance).fill(
            0.02,
            0.02,
            0.01
        ),
        // Global measurement standard deviations. X, Y, and theta.
        MatBuilder(N3.instance, N1.instance).fill(
            0.1,
            0.1,
            0.01
        )
    )

    /**
     * A object with restrictions on how the robot will move
     */
    private val pathingConfig =
        TrajectoryConfig(3.0.metersPerSecond.value, 3.0).apply { // TODO: mess with this
            setKinematics(Drivetrain.kinematics)
        }

    /**
     * Generate a through a list of positions
     *
     * @param waypoints list of Translation2d that the robot should go through
     */
    fun trajectory(waypoints: List<Translation2d>): Trajectory {
        return TrajectoryGenerator.generateTrajectory(
            pose,
            waypoints,
            Pose2d(waypoints.last(), heading), // TODO figure out how to not set heading
            pathingConfig
        )
    }

    /**
     * Generate a command to follow a designated trajectory
     *
     * @param trajectory path for the robot to follow
     */
    private fun ramsete(trajectory: Trajectory): RamseteCommand {
        return RamseteCommand(
            trajectory,
            this::pose,
            RamseteController(Constants.RAMSETE_BETA, Constants.RAMSETE_ZETA),
            Drivetrain.feedforward,
            Drivetrain.kinematics,
            Drivetrain::wheelSpeeds,
            Drivetrain.leftPID,
            Drivetrain.rightPID,
            // RamseteCommand passes volts to the callback
            Drivetrain::driveVolts,
            Drivetrain
        )
    }

    init {
        SmartDashboard.putData("Field", field)
    }

    /**
     * The periodic call of Navigation
     *
     * Used to update the odometry of the robot and log Debug values
     */
    override fun periodic() {
        update()
        field.robotPose = pose
        if (Constants.DEBUG) {  // TODO: figure out what goes here

        }
    }

    // ----- public variables ----- //
    // location
    val heading  // what direction the robot is facing
        get() = Rotation2d(0.0) //RobotContainer.gyro
    var pose  // the location and direction of the robot
        get() = estimator.estimatedPosition
        set(value) = estimator.resetPosition(value, heading)
    val position  // the estimated location of the robot
        get() = pose.translation
    var currentTrajectory: Trajectory? = null
        set(value) {
            if (value != null)
                fieldTraj.setTrajectory(value!!)
            field = value
        }

    // speed
    val leftVel  // how fast the left side of the robot is going
        get() = Drivetrain.leftFront.encoder.velocity
    val rightVel  // how fast the right side of the robot is going
        get() = Drivetrain.rightFront.encoder.velocity
    val wheelSpeeds  // combines wheel speeds
        get() = DifferentialDriveWheelSpeeds(leftVel, rightVel)
    val chassisVel  // general velocity of the robot
        get() = Drivetrain.kinematics.toChassisSpeeds(wheelSpeeds)

    /**
     * Update position based on estimated motion
     */
    private fun update() {  // estimate motion
        estimator.update(heading, wheelSpeeds, leftVel.meters.value, rightVel.meters.value)
    }

    /**
     * Update position based on a different position guess
     *
     * @param globalPosition the detected pose of the Robot
     * @param time the time of the detection
     */
    fun update(globalPosition: Pose2d, time: Double) {  // apply global position update
        estimator.addVisionMeasurement(globalPosition, time)
    }

    // a demo trajectory to follow
    val demo = TrajectoryGenerator.generateTrajectory(
        Pose2d(0.0, 0.0, Rotation2d(0.0)),
        listOf(Translation2d(1.0, 1.0), Translation2d(2.0, -1.0)),
        Pose2d(3.0, 0.0, Rotation2d(0.0)),
        TrajectoryConfig(3.0.feet.value, 3.0.feet.value).apply {
            setKinematics(Drivetrain.kinematics)
        }
    )
}