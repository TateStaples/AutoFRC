package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.drive.AutoDrive
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.inches
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation

class SimTest : KRobot() {
    val leftMotor = KSparkMax(1, MotorType.BRUSHLESS)
    val rightMotor = KSimulatedESC("right")
    val configs = DifferentialDriveConfigs(1.inches, 1.feet)
    val gyro = KPigeon(1)
    val driveTrain = DifferentialDriveTrain(leftMotor, rightMotor, configs, gyro)

    override fun simulationInit() {
        val ff = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
        leftMotor.addFeedforward(ff)
        rightMotor.addFeedforward(ff)
        driveTrain.setupSim(Constants.DRIVE_KV, Constants.DRIVE_KA, 1.5, 0.3)  // this is a physical representation of the drivetrain
        driveTrain.drive(ChassisSpeeds(1.0, 0.0, 0.0))  // starts it driving forward
        Simulation.include(driveTrain)  // this will periodically update
    }

    override fun simulationPeriodic() {

    }
}