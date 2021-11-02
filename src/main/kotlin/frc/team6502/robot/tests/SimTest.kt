package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import frc.team6502.robot.Constants
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.Pose2d
import kyberlib.math.units.Translation2d
import kyberlib.math.units.extensions.*
import kyberlib.math.units.zeroPose
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation

class SimTest : KRobot() {
    private val leftMotor = KSimulatedESC("left")
    private val rightMotor = KSimulatedESC("right")
    private val configs = DifferentialDriveConfigs(2.inches, 1.feet)
    private val gyro = KPigeon(1)
    val driveTrain = DifferentialDriveTrain(leftMotor, rightMotor, configs, gyro)
    val controller = KXboxController(0).apply {
        rightX.apply {
            deadband = 0.1
        }
        leftY.apply {
            deadband = 0.1
        }
    }

    override fun simulationInit() {
        val ff = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
        leftMotor.addFeedforward(ff)
        rightMotor.addFeedforward(ff)
        driveTrain.setupSim(Constants.DRIVE_KV, Constants.DRIVE_KA, 1.5, 0.3)  // this is a physical representation of the drivetrain
        driveTrain.drive(ChassisSpeeds(1.0, 0.0, 0.0))  // starts it driving forward
        Simulation.include(driveTrain)  // this will periodically update
//        driveTrain.pose = Pose2d(2.meters, 2.meters, 0.degrees)
    }

    override fun simulationPeriodic() {
        val forward = -controller.leftY.value
        val turn = -controller.rightX.value
        SmartDashboard.putNumber("forward", forward)
        SmartDashboard.putNumber("turn", turn)
        driveTrain.drive(ChassisSpeeds(forward * Constants.velocity.metersPerSecond, 0.0, turn))
//        driveTrain.drive(ChassisSpeeds(0.0, 0.0, 1.0))

        Simulation.field.robotPose = driveTrain.pose
    }
}