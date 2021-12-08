package frc.team6502.robot.commands.drive

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup
import frc.team6502.robot.auto.cv.Vision
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.subsystems.Shooter
import kyberlib.auto.Navigator
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.transform
import kotlin.math.absoluteValue

class BallHunter: CommandBase() {
    val glowormTable = NetworkTableInstance.getDefault().getTable("photonvision/gloworm")
    val yawEntry = glowormTable.getEntry("targetYaw")
    val yaw: Double
        get() {
//            return Vision.targetResult.bestTarget.yaw
            return yawEntry.getDouble(0.0)
        }

    fun generateMunch(): ParallelDeadlineGroup {
        return ParallelDeadlineGroup(AutoDrive(Navigator.instance!!.pose.transformBy(Pose2d((0.5).meters, (-0.2).meters, (-30).degrees).transform)), Intake())
    }

    val timeSinceLastFound = Timer()
    var everFound = false
    override fun execute() {
        val res = Vision.targetResult
        if (res.hasTargets()) {
            timeSinceLastFound.reset()
            everFound = true
            if (yaw.absoluteValue < 3) Drivetrain.drive(ChassisSpeeds(0.1, 0.0, 0.0))
            else Drivetrain.drive(ChassisSpeeds(0.0, 0.0, -yaw / 20.0))  // yaw in degrees
        } else {
            if (!timeSinceLastFound.hasElapsed(3.0) && everFound  && false) {
                Shooter.intake.voltage = -8.0
                Drivetrain.drive(ChassisSpeeds(0.5, 0.0, -0.5))
            }
            else {
                Drivetrain.stop()
            }
        }
    }


}