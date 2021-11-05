package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.Constants
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.subsystems.Drive
import kyberlib.auto.Navigator
import kyberlib.command.CommandManager
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.zeroPose
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d
import kotlin.math.PI

class Integration : KRobot() {
    init {
        Drive
        if (!Simulation.real) {
            Simulation.instance.include(Drive)
            Drive.setupSim()
        }
    }
    val gyro = KPigeon(Constants.PIGEON_PORT)
    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -3 * PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -2.0
            expo = 20.0
            deadband = 0.2
        }
    }

    val navigation = Navigator(gyro)

    override fun teleopPeriodic() {
        CommandManager.clear()
        Constants.AUTO = false
        val forward = controller.leftY.value
        val turn = controller.rightX.value
        SmartDashboard.putNumber("forward", forward)
        SmartDashboard.putNumber("turn", turn)
        Drive.chassisSpeeds = ChassisSpeeds(forward, 0.0, turn)
    }

    override fun autonomousPeriodic() {
        Constants.AUTO = true
        Navigation.pose = zeroPose
        val traj = navigation.trajectory(Translation2d(1.0,0.0), Translation2d(1.0, 2.0))
        KField2d.getObject("traj").setTrajectory(traj)
//        val drive1 = AutoDrive(traj)  // Drives here
//        val seq = CommandManager.sequence(drive1, Intake())
//        val drive2 = AutoDrive(Translation2d(10.0, 3.0))  // drives here
//        drive1.schedule()
//        CommandManager.clear()
//        CommandManager.enqueue(drive1)
//        CommandManager.enqueue(Intake())
    }

    override fun robotPeriodic() {
        KField2d.robotPose = navigation.pose
    }

    override fun simulationInit() {
//        Drive.defaultCommand = AutoDrive(Translation2d(3.0, 3.0))
    }

    override fun simulationPeriodic() {
    }

}