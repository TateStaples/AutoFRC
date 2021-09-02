package frc.team6502.robot.subsystems

import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team6502.robot.Constants
import frc.team6502.robot.commands.DefaultDrive
import edu.wpi.first.wpilibj.drive.MecanumDrive
import edu.wpi.first.wpilibj.drive.Vector2d
import edu.wpi.first.wpilibj.Spark
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import frc.team6502.robot.APrefrences

object Drivetrain: SubsystemBase() {
    val leftFront = CANSparkMax(Constants.LEFT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    val leftBack  = CANSparkMax(Constants.LEFT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    val leftSide = SpeedControllerGroup(leftFront, leftBack)
    val rightFront  = CANSparkMax(Constants.RIGHT_FRONT_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    val rightBack = CANSparkMax(Constants.RIGHT_BACK_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    val rightSide = SpeedControllerGroup(rightFront, rightBack)
    val robotDrive = MecanumDrive(leftFront, leftBack, rightFront, rightBack)

    val succ = CANSparkMax(Constants.SUCC_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    val shooterMotor = CANSparkMax(Constants.SHOOTER_ID, CANSparkMaxLowLevel.MotorType.kBrushless).apply {
        idleMode = CANSparkMax.IdleMode.kBrake
    }
    // val robotDrive = DifferentialDrive(leftSide,rightSide)

    init {
        this.defaultCommand = DefaultDrive()
//        val motorList = [leftFront, leftBack, rightFront, rightBack]
        leftFront.inverted = false
        leftBack.inverted = false
        rightFront.inverted = false
        rightBack.inverted = false
        if (APrefrences.DebugMotors) {
            robotDrive.toString()
        }
    }

    var frontIsFront = 1

    fun switchFrontIsFront(){
        frontIsFront*=-1
    }
//    fun ToggleBoost(){
//        Boost = !Boost
//    }

    fun drive ( speed: Vector2d, rotation:Double){
        if (APrefrences.ControllerPositions) {
            println("x: " + speed.x.toString() + " y: " + speed.y.toString())
        }
        var x = 0.0
        var y = 0.0
        var rot = 0.0
        succ.inverted = true
        if (APrefrences.Forward && APrefrences.Backward) {
            x = speed.x
        } else if (APrefrences.Forward) {
            if (speed.x > 0) {
                x = speed.x
            }
        }
        if (APrefrences.Strafe) {
            y = speed.y
        }
        if (APrefrences.Turning) {
            rot = rotation
        }

        robotDrive.driveCartesian(
            -y,
            -x,
            rot
        )
    }

}