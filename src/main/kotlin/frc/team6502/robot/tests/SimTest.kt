package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.commands.drive.AutoDrive
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.inches
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation

class SimTest : KRobot() {
    val leftMotor = KSimulatedESC("left")
    val rightMotor = KSimulatedESC("right")
    val configs = DifferentialDriveConfigs(1.inches, 1.feet)
    val gyro = KPigeon(1)
    val driveTrain = DifferentialDriveTrain(leftMotor, rightMotor, configs, gyro)

    override fun simulationInit() {
        driveTrain.setupSim(1.98, 0.2, 1.5, 0.3)  // this is a physical representation of the drivetrain
        driveTrain.drive(ChassisSpeeds(1.0, 0.0, 0.0))  // starts it driving forward
        Simulation.include(driveTrain)  // this will periodically update
    }

    override fun simulationPeriodic() {
        println("estimated pose - ${driveTrain.pose}")  // check on the progress
    }
}