package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim
import edu.wpi.first.wpilibj.system.plant.DCMotor
import edu.wpi.first.wpilibj.system.plant.LinearSystemId
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpiutil.math.VecBuilder
import frc.team6502.robot.Constants
import frc.team6502.robot.RobotContainer
import kyberlib.auto.Navigator
import kyberlib.command.Debug
import kyberlib.math.units.debugValues
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.simulation.Simulatable
import kotlin.math.absoluteValue

object Drivetrain : SubsystemBase(), Simulatable, Debug {

    /**
     * A forward projection of how much voltage to apply to get desired velocity and acceleration
     */
    private val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)

    // motors
    val leftMaster = KSparkMax(Constants.LEFT_FRONT_ID, MotorType.BRUSHLESS).apply {
        identifier = "leftMaster"
        brakeMode = true
        reversed = false
        currentLimit = 40
    }
    val rightMaster  = KSparkMax(Constants.RIGHT_FRONT_ID, MotorType.BRUSHLESS).apply {
        identifier = "rightMaster"
        brakeMode = true
        reversed = true
        currentLimit = 40
    }
    private val leftFollower  = KSparkMax(Constants.LEFT_BACK_ID, MotorType.BRUSHLESS).apply {
        identifier = "leftFollow"
        brakeMode = true
        reversed = false
        currentLimit = 40
        follow(leftMaster)
    }
    private val rightFollower = KSparkMax(Constants.RIGHT_BACK_ID, MotorType.BRUSHLESS).apply {
        identifier = "rightFollow"
        reversed = false
        currentLimit = 40
        follow(rightMaster)
    }
    private val motors = arrayOf(leftMaster, rightMaster)

    /**
     * Configure all the encoders with proper gear ratios
     */
    init {
//        defaultCommand = CommandManager
        for (motor in motors) {
            motor.apply {
                brakeMode = true
                gearRatio = Constants.DRIVE_GEAR_RATIO
                radius = Constants.WHEEL_RADIUS
                currentLimit = 40

                kP = Constants.DRIVE_P
                kI = Constants.DRIVE_I
                kD = Constants.DRIVE_D

                addFeedforward(feedforward)
            }
        }
    }

    /**
     * Do important math specific to your chassis
     */
    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    /**
     * A list of important variables the rest of the code needs easy access to
     */
    var wheelSpeeds: DifferentialDriveWheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftMaster.linearVelocity.metersPerSecond, rightMaster.linearVelocity.metersPerSecond)
        set(value) {drive(value)}
    var chassisSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(wheelSpeeds)
        set(value) {drive(value)}

    /**
     * Drive the robot in a specific direction
     * @param speeds the velocity you want the robot to start moving
     */
    fun drive(speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    /**
     * Drive the robot with specific wheel speeds
     * @param speeds DifferentialDriveWheelsSpeeds that have instruction for how fast each side should go
     */
    private fun drive(speeds: DifferentialDriveWheelSpeeds) {
        speeds.normalize(Constants.velocity.metersPerSecond)
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = speeds.rightMetersPerSecond.metersPerSecond
    }

    fun stop() {
        leftMaster.voltage = 0.0
        rightMaster.voltage = 0.0
    }

    override fun periodic() {
        RobotContainer.navigation.update(chassisSpeeds)
        debugDashboard()
    }

    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim(KvAngular: Double = 5.5, KaAngular: Double = 0.5, KvLinear: Double = Constants.DRIVE_KV, KaLinear: Double = Constants.DRIVE_KA,) {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            leftMaster.gearRatio,  // gearing reduction
            kinematics.trackWidthMeters,  // The track width
            leftMaster.radius!!.meters,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
    }

    private fun roundLows(d: Double): Double = if (d.absoluteValue < 0.05) 0.0 else d

    override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        val leftVolt = leftMaster.voltage
        val rightVolt = rightMaster.voltage
        driveSim.setInputs(roundLows(leftVolt), roundLows(rightVolt))
        driveSim.update(dt)

        // update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        Navigator.instance!!.heading = driveSim.heading.k
    }

    override fun debugValues(): Map<String, Any?> {
        return mapOf(
            "pose" to RobotContainer.navigation.pose.debugValues,
            "speed" to chassisSpeeds.debugValues,
            "leftMaster" to leftMaster,
            "rightMaster" to rightMaster,
            "leftFollow" to leftFollower,
            "rightFollow" to rightFollower
        )
    }
}