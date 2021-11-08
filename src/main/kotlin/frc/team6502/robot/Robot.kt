package frc.team6502.robot

import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.drive.DefaultDrive
import frc.team6502.robot.commands.general.Strategy
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.command.CommandManager
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.degrees
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d

class Robot : KRobot() {
    init {
        Drivetrain
        if (!Simulation.real) {
            Simulation.instance.include(Drivetrain)
            Drivetrain.setupSim()
        }
    }

    override fun teleopInit() {
        Constants.AUTO = false
        CommandManager.clear()
        KField2d.trajectory = null
        CommandManager.enqueue(DefaultDrive())
    }

    override fun autonomousInit() {
        Constants.AUTO = true
        CommandManager.clear()
        Strategy.plan()
    }

    override fun robotInit() {
        RobotContainer.navigation.pose = Pose2d(0.1, 0.1, 0.degrees)
    }
    override fun robotPeriodic() {
        KField2d.robotPose = RobotContainer.navigation.pose
    }

    override fun simulationInit() {
        for (x in arrayOf(1.0, 2.0, 3.0)) {
            for (y in arrayOf(1.0, 2.0))
                KField2d.addGoal(Translation2d(x, y), 0.0, "ball", Intake())
        }
    }
}