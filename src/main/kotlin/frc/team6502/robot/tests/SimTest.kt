package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.drive.AutoDrive
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.Pose2d
import kyberlib.math.units.extensions.degrees
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation

class SimTest : KRobot() {
    val leftMotor = KSimulatedESC("left")
    val rightMotor = KSimulatedESC("right")
    val configs = DifferentialDriveConfigs(2.inches, 0.657299651.meters)
    val gyro = KPigeon(1)
    val driveTrain = DifferentialDriveTrain(leftMotor, rightMotor, configs, gyro)
//    val controller = KXboxController(0)  // todo

    override fun simulationInit() {
        val ff = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
        leftMotor.addFeedforward(ff)
        rightMotor.addFeedforward(ff)
        driveTrain.setupSim(Constants.DRIVE_KV, Constants.DRIVE_KA, 1.5, 0.3)  // this is a physical representation of the drivetrain
        driveTrain.drive(ChassisSpeeds(1.0, 0.0, 0.0))  // starts it driving forward
        Simulation.include(driveTrain)  // this will periodically update
        driveTrain.pose = Pose2d(2.meters, 2.meters, 0.degrees)
    }

    override fun simulationPeriodic() {
//        val forward = controller.leftY.value
//        val turn = controller.rightX.value
//        println("forward: $forward, turn: $turn")
//        driveTrain.drive(ChassisSpeeds(forward, 0.0, turn))
    }
}