package frc.team6502.robot

import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.KRobot


/**
 * Main robot class. Runs the main control loops
 */
class Robot : KRobot() {

    override fun robotInit() {
        // initialize RobotContainer and by extension subsystems
        RobotContainer
    }

    override fun robotPeriodic() {

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
