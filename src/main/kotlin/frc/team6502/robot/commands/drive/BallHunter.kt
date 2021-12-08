package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.auto.cv.Vision
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.auto.Navigator
import kyberlib.math.units.extensions.radians

class BallHunter: CommandBase() {
    private var munch = false

    override fun execute() {
        val res = Vision.targetResult
        if (res.hasTargets()) {
            val imageCaptureTime = Timer.getFPGATimestamp() - res.latencyMillis
            val target = res.targets.first()
            Drivetrain.faceDirection(Navigator.instance!!.heading + target.yaw.radians)
        } else {
            Drivetrain.drive(ChassisSpeeds(0.0, 0.0, 1.0))
        }
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }


}