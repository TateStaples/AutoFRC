package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.general.CommandManager
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

    override fun autonomousInit() {
        Constants.AUTO = true
        val drive1 = AutoDrive(Translation2d(3.0, 0.0))  // Drives here
        val drive2 = AutoDrive(Translation2d(10.0, 3.0))  // drives here
        CommandManager.enqueue(CommandManager.sequence(drive1, drive2))
    }

    override fun teleopInit() {
        Constants.AUTO = false
        Navigation.pose = zeroPose
    }
}
