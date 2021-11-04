package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.balls.Shoot
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.extensions.meters
import kyberlib.mechanisms.drivetrain.DifferentialDriveConfigs
import kyberlib.mechanisms.drivetrain.DifferentialDriveTrain
import kyberlib.motorcontrol.KMotorController
import kyberlib.motorcontrol.MotorType
import kyberlib.motorcontrol.rev.KSparkMax
import kyberlib.sensors.gyros.KPigeon
import kotlin.math.PI

class MotorControlTest: KRobot() {
    private val gyro = KPigeon(Constants.PIGEON_PORT)

    /**
     * The main user input device of robot
     */
    val controller = KXboxController(0).apply {
        rightX.apply {
            rate = -5 * PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            rate = -12.0
            expo = 20.0
            deadband = 0.2
        }

        rightBumper.whileActiveOnce(Shoot())
        leftBumper.whileActiveOnce(Intake())
    }

    val feedforward = SimpleMotorFeedforward(Constants.DRIVE_KS, Constants.DRIVE_KV, Constants.DRIVE_KA)
    val frontLeftMotor = KSparkMax(Constants.LEFT_FRONT_ID, MotorType.BRUSHLESS).apply {
        addFeedforward(feedforward)
        radius = Constants.WHEEL_RADIUS
        gearRatio = 1.0/20.0
        identifier = "left"
    }
    val frontRightMotor = KSparkMax(Constants.RIGHT_FRONT_ID, MotorType.BRUSHLESS).apply {
        addFeedforward(feedforward)
        radius = Constants.WHEEL_RADIUS
        gearRatio = 1.0/20.0
        identifier = "right"
        reversed = true
    }
    val backLeftMotor = KSparkMax(Constants.LEFT_BACK_ID, MotorType.BRUSHLESS).apply {
        follow(frontLeftMotor)
        radius = Constants.WHEEL_RADIUS
        gearRatio = 1.0/20.0
    }
    val backRightMotor = KSparkMax(Constants.RIGHT_BACK_ID, MotorType.BRUSHLESS).apply {
        follow(frontRightMotor)
        radius = Constants.WHEEL_RADIUS
        gearRatio = 1.0/20.0
    }
    val leftMotors = arrayOf<KMotorController>(frontLeftMotor, backLeftMotor)
    val rightMotors = arrayOf<KMotorController>(frontRightMotor, backRightMotor)

    val driveConfig = DifferentialDriveConfigs(Constants.WHEEL_RADIUS, Constants.TRACK_WIDTH.meters)
    val drivetrain = DifferentialDriveTrain(leftMotors, rightMotors, driveConfig, gyro)

    override fun teleopPeriodic() {
        val forward = controller.leftY.value.coerceAtMost(1.0)
        val turn = controller.rightX.value.coerceAtMost(1.0)
        println("Controls - $forward, $turn")
        val speeds = ChassisSpeeds(forward, 0.0, turn)
        drivetrain.drive(speeds)
        frontLeftMotor.debugDashboard()
        frontRightMotor.debugDashboard()
    }

    override fun autonomousPeriodic() {
        val speeds = ChassisSpeeds(1.0, 0.0, 0.0)
        drivetrain.drive(speeds)
    }
}