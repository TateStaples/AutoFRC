package frc.team6502.robot.commands.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.auto.cv.Vision

class BallHunter: CommandBase() {
    override fun execute() {
        val res = Vision.targetResult
        if (res.hasTargets()) {
            val imageCaptureTime = Timer.getFPGATimestamp() - res.latencyMillis
            for (target in res.targets) {

            }
        }
    }

    override fun isFinished(): Boolean {
        return super.isFinished()
    }


}