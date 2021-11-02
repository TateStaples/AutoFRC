package kyberlib.mechanisms.drivetrain

import edu.wpi.first.wpilibj.geometry.Pose2d
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
import kyberlib.math.units.speed
import kyberlib.motorcontrol.KMotorController
import kyberlib.sensors.gyros.KGyro
import kyberlib.simulation.Simulatable
import kyberlib.simulation.Simulation
import java.lang.Math.cos
import kotlin.math.sin

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

    var pose: Pose2d
        set(value) {
            odometry.resetPosition(value, gyro.heading)
        }
        get() = odometry.poseMeters

    override fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = speeds.rightMetersPerSecond.metersPerSecond
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
        val chassis = ChassisSpeeds(1.0, 0.0, 0.5)
        drive(chassis)
        leftMaster.voltage = leftMaster.customControl!!(leftMaster)
        rightMaster.voltage = rightMaster.customControl!!(rightMaster)
        driveSim.setInputs(leftMaster.voltage, rightMaster.voltage)
        driveSim.update(dt)

        leftMaster.resetPosition(driveSim.leftPositionMeters.meters)
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.resetPosition(driveSim.rightPositionMeters.meters)
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond

        val sped = kinematics.toChassisSpeeds(DifferentialDriveWheelSpeeds(driveSim.leftVelocityMetersPerSecond, driveSim.rightVelocityMetersPerSecond))
        gyro.heading = (-driveSim.heading).k
        odometry.update(driveSim.heading, driveSim.leftPositionMeters, driveSim.rightPositionMeters)

        leftMaster.debugDashboard()
        rightMaster.debugDashboard()
        SmartDashboard.putNumber("Velocity (x)", sped.speed.metersPerSecond * kotlin.math.cos(pose.rotation.radians))
        SmartDashboard.putNumber("Velocity (y)", sped.speed.metersPerSecond * sin(pose.rotation.radians))
        SmartDashboard.putNumber("X", pose.x)
        SmartDashboard.putNumber("Y", pose.y)
        SmartDashboard.putNumber("THETA", pose.rotation.radians)
        field.robotPose = pose
//        if (pose.translation.getDistance(prevPose.translation) > 3.0) 0/0
    }
}
