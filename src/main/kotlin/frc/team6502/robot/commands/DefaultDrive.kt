package frc.team6502.robot.commands

import edu.wpi.first.wpilibj.drive.Vector2d
import frc.team6502.robot.Helpers.snap
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.team6502.robot.APrefrences
import frc.team6502.robot.Constants
import frc.team6502.robot.OI
import frc.team6502.robot.subsystems.Drivetrain
import edu.wpi.first.wpilibj.GenericHID.*
import kotlin.math.*

class DefaultDrive: CommandBase() {


    init {
        addRequirements(Drivetrain)
    }
    override fun initialize() {

    }

    override fun execute() {
//        SmartDashboard.putNumber("Y AXIS:" , OI.controllerLY)
//        SmartDashboard.putNumber("X AXIS:" , OI.controllerLX)
        try {
            SmartDashboard.putNumber("y", OI.controllerLY)
            SmartDashboard.putNumber("x", OI.controllerLX)
        } catch (e: Exception) {
            e.printStackTrace()
        }
        var control = Vector2d(
            OI.controllerLY * APrefrences.GeneralSpeed,// * Drivetrain.frontIsFront,// * Constants.MAX_SPEED * OI.controllerThrottle,
            OI.controllerLX * APrefrences.GeneralSpeed// * Drivetrain.frontIsFront// * Constants.MAX_SPEED * OI.controllerThrottle
        )
        var turn = OI.controllerRX * APrefrences.GeneralSpeed
/*        if (true) {*/
        if (OI.XBControll.getBumper(Hand.kRight)) {
            Drivetrain.shooterMotor.set(1 * APrefrences.shooterMultiplier)
            if (APrefrences.Rumble) {
                OI.XBControll.setRumble(RumbleType.kRightRumble, -(Drivetrain.shooterMotor.get()-APrefrences.shooterMultiplier))
            }
        } else {
            OI.XBControll.setRumble(RumbleType.kRightRumble,0.0)
            Drivetrain.shooterMotor.set(-APrefrences.ReverseIntakeSpeed * APrefrences.shooterMultiplier)
        }
        if (OI.XBControll.getBumper(Hand.kLeft) && !(OI.XBControll.bButtonPressed)) {
            Drivetrain.succ.set(1 * APrefrences.shooterMultiplier)
            if (APrefrences.Rumble) {
                OI.XBControll.setRumble(RumbleType.kLeftRumble, -(Drivetrain.shooterMotor.get()-APrefrences.shooterMultiplier))
            }
        } else if (OI.XBControll.bButton) {
            Drivetrain.succ.set(-APrefrences.ReverseIntakeSpeed * APrefrences.intakeMultiplier)
        } else {
            OI.XBControll.setRumble(RumbleType.kLeftRumble,0.0)
            Drivetrain.succ.set(0.0)
            Drivetrain.succ.set(0.0)
        }
        control = snap(Vector2d((control.x*abs(control.x)), (control.y*abs(control.y))))
        Drivetrain.drive(control, turn)/*
        }*//* else {
//            Drivetrain.drive(OI.controllerLY * Drivetrain.frontIsFront * Constants.MAX_SPEED, OI.controllerLX * Constants.MAX_SPEED * Drivetrain.frontIsFront)
        }*/
    }

    override fun isFinished() = false
}