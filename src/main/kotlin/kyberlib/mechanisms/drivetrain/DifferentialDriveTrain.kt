package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpiutil.math.VecBuilder
import kyberlib.math.units.extensions.*
import kyberlib.motorcontrol.KMotorController
import kyberlib.sensors.gyros.KGyro
import kyberlib.simulation.Simulatable

/**
 * Stores important information for the motion of a DifferentialDrive Robot
 */
data class DifferentialDriveConfigs(val wheelRadius: Length, val trackWidth: Length)

/**
 * Pre-made DifferentialDrive Robot.
 * @param leftMotors array of all the motors on the left side of the robot. They will all follow the first
 * @param rightMotors array of all the motors on the right side of the robot. They will all follow the first
 * @param configs information about the physical desciption of this drivetrain
 * @param gyro KGyro to provide heading information
 */
class DifferentialDriveTrain(leftMotors: Array<KMotorController>, rightMotors: Array<KMotorController>,
                             private val configs: DifferentialDriveConfigs, val gyro: KGyro) : SubsystemBase(), Simulatable,
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

    val pose
        get() = odometry.poseMeters

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

    private lateinit var driveSim: DifferentialDrivetrainSim
    private lateinit var field: Field2d
    fun setupSim(KvLinear: Double, KaLinear: Double, KvAngular: Double, KaAngular: Double) {
        field = Field2d()
        SmartDashboard.putData("Field", field)
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            motors.first().gearRatio,  // gearing reduction
            configs.trackWidth.meters,  // The track width
            configs.wheelRadius.meters,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
    }

    override fun simUpdate(dt: Double) {
//        drive(ChassisSpeeds(1.0, 0.0, 0.0))
        val leftVolt = 1.0//leftMaster.customControl!!(leftMaster)
        val rightVolt = 1.0//leftMaster.customControl!!(leftMaster)
//        leftMaster.voltage = leftVolt
//        rightMaster.voltage = rightVolt
        println("${leftMaster.percent}, ${leftMaster.voltage}, ${leftMaster.customControl!!(leftMaster)}")
//        leftMaster.debugPrint()
        leftMaster.debugDashboard()
        driveSim.setInputs(leftVolt, rightVolt)
        driveSim.update(dt)
        leftMaster.linearPosition = driveSim.leftPositionMeters.meters
        leftMaster.linearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.linearPosition = driveSim.rightPositionMeters.meters
        rightMaster.linearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        gyro.heading = (-driveSim.heading).k
        odometry.update(driveSim.heading, driveSim.leftPositionMeters, driveSim.rightPositionMeters)
        field.robotPose = odometry.poseMeters
    }
}
