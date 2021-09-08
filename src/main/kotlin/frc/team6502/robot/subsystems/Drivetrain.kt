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
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.kyberlib.math.Differentiator
import frc.team6502.kyberlib.math.units.extensions.metersPerSecond
import frc.team6502.kyberlib.math.units.extensions.radiansPerSecond
import frc.team6502.robot.RobotContainer
import frc.team6502.robot.commands.AutoDrive

object Drivetrain: SubsystemBase() {
    // motors
    val leftFront = CANSparkMax(Constants.LEFT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
    }
    val rightFront  = CANSparkMax(Constants.RIGHT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
    }

    val leftBack  = CANSparkMax(Constants.LEFT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
        follow(leftFront)
    }
    val rightBack = CANSparkMax(Constants.RIGHT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        restoreFactoryDefaults()
        idleMode = CANSparkMax.IdleMode.kBrake
        inverted = false
        setSmartCurrentLimit(40)
        follow(rightFront)
    }

    // controls
    val kinematics = DifferentialDriveKinematics(Constants.TRACK_WIDTH)

    val leftAccelCalculator = Differentiator()
    val rightAccelCalculator = Differentiator()

    val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS_L, Constants.DRIVE_KV_L, Constants.DRIVE_KA_L)

    val leftPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)
    val rightPID = PIDController(Constants.DRIVE_P, Constants.DRIVE_I, Constants.DRIVE_D)

    init {
        if (Constants.AUTO) defaultCommand = AutoDrive() else DefaultDrive()
    }

    // public vars
    val leftVel
        get() = leftFront.encoder.velocity
    val rightVel
        get() = rightFront.encoder.velocity
    val wheelSpeeds
        get() = DifferentialDriveWheelSpeeds(leftVel, rightVel)

    fun drive (speeds: ChassisSpeeds) { drive(kinematics.toWheelSpeeds(speeds))}

    fun drive(speeds: DifferentialDriveWheelSpeeds) {
        val leftSpeed = speeds.leftMetersPerSecond
        val rightSpeed = speeds.rightMetersPerSecond

        val lPID = leftPID.calculate(leftFront.encoder.velocity, leftSpeed)
        val lFF = feedforward.calculate(leftSpeed, leftAccelCalculator.calculate(leftSpeed))
        val rPID = rightPID.calculate(rightFront.encoder.velocity, rightSpeed)
        val rFF = feedforward.calculate(rightSpeed, rightAccelCalculator.calculate(rightSpeed))
        if (Constants.DEBUG) {
            debug()
        }
//        driveVo(lPID + lFF, rPID + rFF)

        leftFront.set(leftSpeed)
        rightFront.set(rightSpeed)
    }

    fun driveVolts(leftVolts: Double, rightVolts: Double) {
        leftFront.setVoltage(leftVolts)
        rightFront.setVoltage(rightVolts)
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
//        leftPID.initSendable()

        logEncoders()

    }

}