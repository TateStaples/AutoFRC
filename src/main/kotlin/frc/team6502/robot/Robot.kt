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
import frc.team6502.robot.Auto.Navigation
import frc.team6502.robot.subsystems.Drivetrain


/**
 * Main robot class. Runs the main control loops
 */
class Robot : TimedRobot() {

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
        val trajectory = RobotContainer.navigation.trajectory(arrayListOf(Translation2d(3.0, 0.0)))
        RobotContainer.navigation.currentTrajectory = trajectory
        val ramseteCommand = RobotContainer.navigation.ramsete(trajectory)

        ramseteCommand.andThen(Runnable{ Drivetrain.driveVolts(0.0, 0.0) })
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {

    }

    override fun teleopPeriodic() {

    }

}
