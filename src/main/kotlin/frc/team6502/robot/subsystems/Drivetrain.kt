package frc.team6502.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.kinematics.*
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.general.CommandManager
import kyberlib.math.Filters.Differentiator
import kyberlib.math.units.extensions.*
import kotlin.math.PI

/**
 * Holds the motors and controllers in charge of moving the robot chassis
 */
object Drivetrain : SubsystemBase() {
    // motors
    private val leftFront = CANSparkMax(Constants.LEFT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
    }
    private val rightFront  = CANSparkMax(Constants.RIGHT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = true
        setSmartCurrentLimit(40)
    }
    private val leftBack  = CANSparkMax(Constants.LEFT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
        if (!Constants.MECANUM) follow(leftFront)
    }
    private val rightBack = CANSparkMax(Constants.RIGHT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = true
        setSmartCurrentLimit(40)
        if (!Constants.MECANUM) follow(rightFront)
    }
    private val motors = arrayOf(leftFront, leftBack, rightFront, rightBack)

    /**
     * Configure all the encoders with proper gear ratios
     */
    init {
        val circumference = Constants.WHEEL_RADIUS.meters * 2 * PI
        for (motor in motors) {
            motor.encoder.apply {
                velocityConversionFactor = (circumference * Constants.DRIVE_GEAR_RATIO).rpm.rotationsPerSecond
                positionConversionFactor = circumference * Constants.DRIVE_GEAR_RATIO
            }
            val pid = motor.pidController
            pid.apply {
                p = Constants.DRIVE_P
                i = Constants.DRIVE_I
                d = Constants.DRIVE_D
            }
        }
    }

    /**
     * Location of each of the wheels relative to the center of the robot.
     * Important for mecanum control
     */
    private val robotWidth = 8.686.inches
    private val robotLength = 5.75.inches
    private val frontLeftPosition = Translation2d(-robotWidth.meters, robotLength.meters)
    private val frontRightPosition = Translation2d(robotWidth.meters, robotLength.meters)
    private val backLeftPosition = Translation2d(-robotWidth.meters, -robotLength.meters)
    private val backRightPosition = Translation2d(robotWidth.meters, -robotLength.meters)

    /**
     * Do important math specific to your chassis
     */
    val difKinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    val mecKinematics = MecanumDriveKinematics(frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition)

    /**
     * Keep real time calculation of the acceleration of each side of drivetrain
     */
    private val leftAccelCalculator = Differentiator()
    private val rightAccelCalculator = Differentiator()

    /**
     * A forward projection of how much voltage to apply to get desired velocity and acceleration
     */
    private val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)

    val leftPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)
    val rightPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)
    val rotationPID = ProfiledPIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D, TrapezoidProfile.Constraints(1.0, 1.0))

    /**
     * Setup the default command for the system
     */
    init {
        defaultCommand = CommandManager
    }

    /**
     * A list of important variables the rest of the code needs easy access to
     */
    private val leftFrontVel get() = leftFront.encoder.velocity
    private val rightFrontVel get() = rightFront.encoder.velocity
    private val leftBackVel get() = leftBack.encoder.velocity
    private val rightBackVel get() = rightBack.encoder.velocity
    val leftVel get() = leftFrontVel
    val rightVel get() = rightFrontVel
    val leftPos get() = leftFront.encoder.position
    val rightPos get() = rightFront.encoder.position
    var difWheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftVel, rightVel)
        set(value) {drive(value)}
    var mecWheelSpeeds
        get() = MecanumDriveWheelSpeeds(leftFrontVel, rightFrontVel, leftBackVel, rightBackVel)
        set(value) {drive(value)}
    var chassisSpeeds: ChassisSpeeds
        get() = if (Constants.MECANUM) mecKinematics.toChassisSpeeds(mecWheelSpeeds)
                else difKinematics.toChassisSpeeds(difWheelSpeeds)
        set(value) {drive(value)}

    /**
     * Drive the robot in a specific direction
     * @param speeds the velocity you want the robot to start moving
     */
    fun drive (speeds: ChassisSpeeds) {
        if (Constants.DEBUG) debug()
        if (Constants.MECANUM) drive(mecKinematics.toWheelSpeeds(speeds))
        else drive(difKinematics.toWheelSpeeds(speeds))
    }

    /**
     * Drive the robot with specific wheel speeds
     * @param speeds DifferentialDriveWheelsSpeeds that have instruction for how fast each side should go
     */
    private fun drive(speeds: DifferentialDriveWheelSpeeds) {
        val leftSpeed = speeds.leftMetersPerSecond
        val rightSpeed = speeds.rightMetersPerSecond

        val lPID = leftPID.calculate(leftFront.encoder.velocity, leftSpeed)
        val lFF = feedforward.calculate(leftSpeed, leftAccelCalculator.calculate(leftSpeed))
        val rPID = rightPID.calculate(rightFront.encoder.velocity, rightSpeed)
        val rFF = feedforward.calculate(rightSpeed, rightAccelCalculator.calculate(rightSpeed))
        if (Constants.DEBUG)
            debug()
//        driveVolts(lPID + lFF, rPID + rFF)

//        leftFront.set(leftSpeed)
//        rightFront.set(rightSpeed)
        leftFront.setVoltage(lPID + lFF)
        rightFront.setVoltage(rPID + rFF)
    }

    /**
     * Drive a Mecanum robot at specific speeds
     * @param speeds the wheel speeds to move the Mecanum robot
     */
    private fun drive(speeds: MecanumDriveWheelSpeeds) {
        speeds.normalize(Constants.velocity.metersPerSecond)
        leftFront.set(speeds.frontLeftMetersPerSecond)
        leftBack.set(speeds.rearLeftMetersPerSecond)
        rightFront.set(speeds.frontRightMetersPerSecond)
        rightBack.set(speeds.rearRightMetersPerSecond)
    }

    /**
     * Display the important values of all the encoders.
     *
     * Values: Position, Velocity, Voltage, Current, inverted
     */
    private fun logEncoders() {
        val names = arrayListOf("Left Front", "Left Back", "Right Front", "Right Back")
        for (info in motors.zip(names)) {
            val motor = info.first
            val encoder = motor.encoder
            val name = info.second
            SmartDashboard.putNumber("$name Position", encoder.position)
            SmartDashboard.putNumber("$name Velocity", encoder.velocity)
            SmartDashboard.putNumber("$name Voltage", motor.appliedOutput)
            SmartDashboard.putNumber("$name Current", motor.outputCurrent)
            SmartDashboard.putBoolean("$name Inverted", encoder.inverted)
        }
    }

    /**
     * Log important debugging values
     */
    private fun debug() {
        SmartDashboard.putNumber("Left Error", leftPID.positionError)
        SmartDashboard.putNumber("Right Error", rightPID.positionError)
//        leftPID.initSendable()

        logEncoders()

    }
}