package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.DefaultDrive
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.kyberlib.math.Differentiator
import frc.team6502.kyberlib.math.units.extensions.metersPerSecond
import frc.team6502.kyberlib.math.units.extensions.radiansPerSecond

object Drivetrain: SubsystemBase() {
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
        inverted = false
        setSmartCurrentLimit(40)
    }

    private val leftBack  = CANSparkMax(Constants.LEFT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
        follow(leftFront)
    }
    private val rightBack = CANSparkMax(Constants.RIGHT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
        follow(rightFront)
    }

    // controls
    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)
    val odometry = DifferentialDriveOdometry(Rotation2d())

    val leftAccelCalculator = Differentiator()
    val rightAccelCalculator = Differentiator()

    private val leftFeedforward = SimpleMotorFeedforward(Constants.DRIVE_KS_L, Constants.DRIVE_KV_L, Constants.DRIVE_KA_L)
    private val rightFeedforward = SimpleMotorFeedforward(Constants.DRIVE_KS_R, Constants.DRIVE_KV_R, Constants.DRIVE_KA_R)

    private val leftPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)
    private val rightPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)

    init {
        defaultCommand = DefaultDrive()
    }

    fun drive (speeds: ChassisSpeeds) { drive(kinematics.toWheelSpeeds(speeds))}

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        val leftSpeed = speeds.leftMetersPerSecond
        val rightSpeed = speeds.rightMetersPerSecond

        val lPID = leftPID.calculate(leftFront.encoder.velocity, leftSpeed)
        val lFF = leftFeedforward.calculate(leftSpeed, leftAccelCalculator.calculate(leftSpeed))
        val rPID = rightPID.calculate(rightFront.encoder.velocity, rightSpeed)
        val rFF = rightFeedforward.calculate(rightSpeed, rightAccelCalculator.calculate(rightSpeed))

//        leftFront.setVoltage(lPID + lFF)
//        rightFront.setVoltage(rPID + rFF)

        leftFront.set(speeds.leftMetersPerSecond)
        rightFront.set(speeds.rightMetersPerSecond)
    }

    fun logEncoders() {
        val motors = arrayListOf(leftFront, leftBack, rightFront, rightBack)
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

    fun debug() {
        SmartDashboard.putNumber("Left Error", leftPID.positionError)
        SmartDashboard.putNumber("Right Error", rightPID.positionError)

        logEncoders()

    }

}