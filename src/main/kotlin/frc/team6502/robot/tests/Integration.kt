package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team6502.robot.Constants
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.Zap
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d
import kotlin.math.PI

class Integration : KRobot() {
    init {
        Drivetrain
        if (!Simulation.real) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
        }
    }

    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -2.0
            expo = 20.0
            deadband = 0.2
        }

        if (!Simulation.real) {
            Trigger { rightTrigger.raw() > 0.5 }.whileActiveOnce(Zap())
        }
    }

    val navigation = Navigation

    override fun teleopPeriodic() {
//        CommandManager.clear()
        Constants.AUTO = false
        val forward = controller.leftY.value
        val turn = controller.rightX.value
        SmartDashboard.putNumber("forward", forward)
        SmartDashboard.putNumber("turn", turn)
        Drivetrain.drive(ChassisSpeeds(forward, 0.0, turn))
    }

    override fun autonomousInit() {
        Constants.AUTO = true
        val traj = navigation.trajectory(Translation2d(1.0,0.0), Translation2d(1.0, 2.0))
        val drive1 = AutoDrive(traj)  // Drives here
//        CommandManager.enqueue(drive1)
        drive1.schedule()
    }

    override fun robotPeriodic() {
        KField2d.robotPose = navigation.pose
    }
}