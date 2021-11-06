package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.drive.DefaultDrive
import kyberlib.command.CommandManager
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
        Navigation.pose = zeroPose
        val traj = Navigation.trajectory(Translation2d(1.0,0.0), Translation2d(1.0, 2.0))
        val drive1 = AutoDrive(traj)  // Drives here
//        val seq = CommandManager.sequence(drive1, Intake())
//        val drive2 = AutoDrive(Translation2d(10.0, 3.0))  // drives here
        CommandManager.clear()
        CommandManager.enqueue(drive1)
        CommandManager.enqueue(Intake())
    }

    override fun teleopInit() {
        Constants.AUTO = false
        Navigation.pose = zeroPose
        CommandManager.clear()
        CommandManager.enqueue(DefaultDrive)
    }
}