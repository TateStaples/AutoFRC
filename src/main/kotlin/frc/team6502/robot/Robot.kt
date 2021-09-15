package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.CommandManager
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
        val trajectory = RobotContainer.navigation.trajectory(Translation2d(3.0, 0.0))
        CommandManager.enqueue(trajectory)
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {

    }

    override fun teleopPeriodic() {

    }

}
