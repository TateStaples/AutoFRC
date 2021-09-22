package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.commands.CommandManager
import kyberlib.command.KRobot
import kyberlib.math.units.zeroPose


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
        Constants.AUTO = true
        val trajectory = RobotContainer.navigation.trajectory(Translation2d(3.0, 0.0), Translation2d(10.0, 3.0))
        CommandManager.enqueue(trajectory)
    }

    override fun autonomousPeriodic() {

    }

    override fun teleopInit() {
        Constants.AUTO = false
        RobotContainer.navigation.pose = zeroPose
    }

    override fun teleopPeriodic() {

    }

}
