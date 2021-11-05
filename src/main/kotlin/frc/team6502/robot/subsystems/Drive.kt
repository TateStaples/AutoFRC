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
import kyberlib.auto.Navigator
import kyberlib.command.CommandManager
import kyberlib.math.units.extensions.k
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.simulation.Simulatable

object Drive : SubsystemBase(), Simulatable {

    /**
     * A forward projection of how much voltage to apply to get desired velocity and acceleration
     */
    private val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)

    // motors
    val leftMaster = KSparkMax(Constants.LEFT_FRONT_ID, MotorType.BRUSHLESS).apply {
        brakeMode = true
        reversed = false
        currentLimit = 40
    }
    val rightMaster  = KSparkMax(Constants.RIGHT_FRONT_ID, MotorType.BRUSHLESS).apply {
        brakeMode = true
        reversed = true
        currentLimit = 40
    }
    private val leftFollower  = KSparkMax(Constants.LEFT_BACK_ID, MotorType.BRUSHLESS).apply {
        brakeMode = true
        reversed = false
        currentLimit = 40
        follow(leftMaster)
    }
    private val rightFollower = KSparkMax(Constants.RIGHT_BACK_ID, MotorType.BRUSHLESS).apply {
        reversed = true
        currentLimit = 40
        follow(rightMaster)
    }
    private val motors = arrayOf(leftMaster, rightMaster)

    /**
     * Configure all the encoders with proper gear ratios
     */
    init {
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
     * Setup the default command for the system
     */
    init {
        defaultCommand = CommandManager
    }

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
    fun drive (speeds: ChassisSpeeds) {
        drive(kinematics.toWheelSpeeds(speeds))
    }

    /**
     * Drive the robot with specific wheel speeds
     * @param speeds DifferentialDriveWheelsSpeeds that have instruction for how fast each side should go
     */
    private fun drive(speeds: DifferentialDriveWheelSpeeds) {
        leftMaster.linearVelocity = speeds.leftMetersPerSecond.metersPerSecond
        rightMaster.linearVelocity = speeds.rightMetersPerSecond.metersPerSecond
    }

    override fun periodic() {
        Navigator.instance!!.update(chassisSpeeds)
    }

    private lateinit var driveSim: DifferentialDrivetrainSim
    fun setupSim(KvAngular: Double = 1.5, KaAngular: Double, KvLinear: Double = Constants.DRIVE_KV, KaLinear: Double = Constants.DRIVE_KA,) {
        driveSim = DifferentialDrivetrainSim( // Create a linear system from our characterization gains.
            LinearSystemId.identifyDrivetrainSystem(KvLinear, KaLinear, KvAngular, KaAngular),
            DCMotor.getNEO(2),  // 2 NEO motors on each side of the drivetrain.
            1.0,  // gearing reduction
            1.0,  // The track width
            0.1,  // wheel radius
            // The standard deviations for measurement noise: x (m), y (m), heading (rad), L/R vel (m/s), L/R pos (m)
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
        )
    }

    override fun simUpdate(dt: Double) {
        // update the sim with new inputs
        driveSim.setInputs(leftMaster.voltage, rightMaster.voltage)
        driveSim.update(dt)

        // update the motors with what they should be
        leftMaster.simLinearPosition = driveSim.leftPositionMeters.meters
        leftMaster.simLinearVelocity = driveSim.leftVelocityMetersPerSecond.metersPerSecond
        rightMaster.simLinearPosition = driveSim.rightPositionMeters.meters
        rightMaster.simLinearVelocity = driveSim.rightVelocityMetersPerSecond.metersPerSecond
        Navigator.instance!!.heading = driveSim.heading.k

        // log the values
        leftMaster.debugDashboard()
        rightMaster.debugDashboard()
    }
}