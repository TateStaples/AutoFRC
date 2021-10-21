package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.controller.HolonomicDriveController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.Constants
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.auto.pathing.PathPlanner
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.towards

class AutoDrive(var targetPose: Pose2d) : CommandBase() {
    constructor(position: Translation2d) : this(Pose2d(position, 0.degrees)) {
        rotationInvariant = true
    }
    private var rotationInvariant = false

    lateinit var trajectory: Trajectory
    private val timer = Timer()

    private companion object Calculations {
        private val difCalculator = RamseteController(Constants.RAMSETE_BETA, Constants.RAMSETE_ZETA)
        private val mecCalculator = HolonomicDriveController(Drivetrain.leftPID, Drivetrain.rightPID, Drivetrain.rotationPID)
    }

    override fun initialize() {
        timer.start()
        if (rotationInvariant)
            targetPose = Pose2d(targetPose.translation, Navigation.position.towards(targetPose.translation))
        trajectory = PathPlanner.pathTo(targetPose.translation)
    }

    override fun execute() {
        if (Constants.MECANUM)
            Drivetrain.chassisSpeeds = mecCalculator.calculate(Navigation.pose, trajectory.sample(timer.get()), targetPose.rotation)
        else
            Drivetrain.chassisSpeeds = difCalculator.calculate(Navigation.pose, trajectory.sample(timer.get()))
//        trajectory = PathPlanner.updateTrajectory(trajectory) - this should be necesary until moving obstabcles
    }

    override fun isFinished(): Boolean {
        return timer.hasElapsed(trajectory.totalTimeSeconds)
    }
}