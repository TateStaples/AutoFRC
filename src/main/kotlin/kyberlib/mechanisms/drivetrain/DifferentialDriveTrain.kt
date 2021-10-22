package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj2.command.SubsystemBase
import kyberlib.motorcontrol.KMotorController
import kyberlib.math.units.extensions.*
import kyberlib.motorcontrol.KSimulatedESC
import kyberlib.sensors.gyros.KGyro


data class DifferentialDriveConfigs(val wheelRadius: Length, val trackWidth: Length)

class DifferentialDriveTrain(leftMotors: Array<KMotorController>, rightMotors: Array<KMotorController>,
                             private val configs: DifferentialDriveConfigs, val gyro: KGyro) : SubsystemBase(),
    Drivetrain {
    constructor(leftMotor: KMotorController, rightMotor: KMotorController,
                configs: DifferentialDriveConfigs, gyro: KGyro) : this(arrayOf(leftMotor), arrayOf(rightMotor), configs, gyro)

    private val motors = leftMotors.plus(rightMotors)
    private val leftMaster = leftMotors[0]
    private val rightMaster = rightMotors[0]


    init {
        for (info in leftMotors.withIndex()) if (info.index > 0) info.value.follow(leftMaster)
        for (info in rightMotors.withIndex()) if (info.index > 0) info.value.follow(rightMaster)
        for (motor in motors)
            motor.radius = configs.wheelRadius
    }

    private val odometry = DifferentialDriveOdometry(0.degrees)
    private val kinematics = DifferentialDriveKinematics(configs.trackWidth.meters)

    override fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
    }

    override fun periodic() {
        odometry.update(gyro.heading, leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond)
    }

    override fun debug() {
        println("${leftMaster.velocity}, ${rightMaster.velocity}")
    }
}
