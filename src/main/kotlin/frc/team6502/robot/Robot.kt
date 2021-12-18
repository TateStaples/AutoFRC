package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.cv.Vision
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.drive.DefaultDrive
import frc.team6502.robot.commands.general.Strategy
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.CommandManager
import kyberlib.command.Game
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.zeroPose
import kyberlib.motorcontrol.KBasicMotorController
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d

class Robot : KRobot() {
    init {
        Drivetrain
        if (Game.sim) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
        }
    }

    override fun disabledInit() {
        CommandManager.clear()
        Drivetrain.stop()
        if (Game.real) {
            KField2d.goals.clear()
        }
        Strategy.collectedBalls = 0
    }

    override fun enabledInit() {
        CommandManager.schedule(false)
        if (Game.real) Vision.cameraOffset = null  // zero position
        RobotContainer.navigation.pose = zeroPose
    }

    override fun teleopInit() {
        CommandManager.clear()
        KField2d.trajectory = null
        CommandManager.enqueue(DefaultDrive)
    }

    override fun autonomousInit() {
        CommandManager.clear()
        Strategy.plan()
    }

    override fun robotPeriodic() {
        KField2d.robotPose = RobotContainer.navigation.pose
    }

    override fun simulationInit() {
        RobotContainer.navigation.pose = Pose2d(0.1, 0.1, 0.degrees)
        for (x in arrayOf(1.0, 2.0, 3.0)) {
            for (y in arrayOf(1.0, 2.0))
                KField2d.addGoal(Translation2d(x, y), 0.0, "ball", Intake())
        }
    }
}