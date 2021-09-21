package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.commands.CommandManager
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.zeroPose
import kyberlib.math.units.zeroTranslation


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
        Drivetrain.odometry.resetPosition(zeroPose, RobotContainer.navigation.heading)
    }

    override fun teleopPeriodic() {

    }

}
