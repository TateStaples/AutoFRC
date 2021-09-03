package frc.team6502.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.RamseteCommand
import frc.team6502.kyberlib.math.units.extensions.feet
import frc.team6502.robot.subsystems.Drivetrain


/**
 * Main robot class.
 */
class Robot : TimedRobot() {

    val field = Field2d()

    override fun robotInit() {
        // report language as kotlin instead of assuming java because of JVM
        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)

        // initialize RobotContainer and by extension subsystems
        RobotContainer
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
    }

    override fun disabledInit() {

    }

    override fun disabledPeriodic() {

    }

    override fun autonomousInit() {
        val trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0.0, 0.0, Rotation2d(0.0)),
            listOf(Translation2d(1.0, 1.0), Translation2d(2.0, -1.0)),
            Pose2d(3.0, 0.0, Rotation2d(0.0)),
            TrajectoryConfig(3.0.feet.value, 3.0.feet.value).apply {
                setKinematics(Drivetrain.kinematics)
            }
        )

        SmartDashboard.putData("Field", field)  // TODO get an image for this
        field.getObject("traj").setTrajectory(trajectory)
        val ramseteCommand = RamseteCommand(
            trajectory,
            Drivetrain::pose,
            RamseteController(Constants.RAMSETE_BETA, Constants.RAMSETE_ZETA),
            Drivetrain.feedforward,
            Drivetrain.kinematics,
            Drivetrain::wheelSpeeds,
            Drivetrain.leftPID,
            Drivetrain.rightPID,
            // RamseteCommand passes volts to the callback
            Drivetrain::drive,
            Drivetrain
        )

        Drivetrain.pose = trajectory.initialPose

        ramseteCommand.andThen(Runnable{ Drivetrain.drive(0.0, 0.0) })
    }

    override fun autonomousPeriodic() {
        field.robotPose = Drivetrain.odometry.poseMeters
    }

    override fun teleopInit() {

    }

    override fun teleopPeriodic() {

    }

}
