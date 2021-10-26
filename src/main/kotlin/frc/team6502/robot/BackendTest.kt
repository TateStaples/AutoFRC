package frc.team6502.robot

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import kyberlib.command.KRobot
import kyberlib.math.units.extensions.feet
import kyberlib.math.units.extensions.inches
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.sensors.gyros.KPigeon

class BackendTest : KRobot() {
    val leftMotor = KSimulatedESC("left")
    val rightMotor = KSimulatedESC("right")
    val configs = DifferentialDriveConfigs(1.inches, 1.feet)
    val gyro = KPigeon(1)
    val driveTrain = DifferentialDriveTrain(leftMotor, rightMotor, configs, gyro)

    override fun simulationInit() {
        driveTrain.setupSim(1.98, 0.2, 1.5, 0.3)
    }

    override fun simulationPeriodic() {
        driveTrain.simUpdate(0.02)
        driveTrain.drive(ChassisSpeeds(1.0, 0.0, 0.0))
    }
}