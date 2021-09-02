package frc.team6502.robot

import edu.wpi.first.wpilibj2.command.button.Trigger
//import frc.team6502.robot.commands.ToggleDirection
//import frc.team6502.robot.commands.ToggleBoost
import frc.team6502.robot.subsystems.Drivetrain
import frc.team6502.robot.APrefrences
import frc.team6502.robot.Helpers.Vector3d
import kotlin.math.abs
import edu.wpi.first.wpilibj.GenericHID.*
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.drive.Vector2d

object OI {
    var XBControll = XboxController(0)
//    val Accelorometerthing = BuiltinAccelerometer()
//    val Accelerometer: Vector3d?
//        get() {
//            if (APrefrences.Accelerometer) {
//                return Vector3d(
//                    BlAccelerometer.getX(),
//                    BlAccelerometer.getY(),
//                    BlAccelerometer.getZ()
//                )
//            } else {
//                return null
//            }
//        }

    val LJoystick: Vector2d
        get() = Vector2d(XBControll.getX(Hand.kLeft), XBControll.getY(Hand.kLeft))
    val RJoystick: Vector2d
        get() = Vector2d(XBControll.getX(Hand.kRight), XBControll.getY(Hand.kRight))
    val controllerThrottle: Double
        get() = 1.0
    val controllerRX: Double
        get() {
            if (APrefrences.RightJoy) {return value(RJoystick.x)} else {return 0.0}
        }
    val controllerRY: Double
        get() {
            if (APrefrences.RightJoy) {return value(-RJoystick.y)} else {return 0.0}
        }
    val controllerLX: Double
        get() {
            if (APrefrences.LeftJoy) {return value(LJoystick.x)} else {return 0.0}
        }
    val controllerLY: Double
        get() {
            if (APrefrences.LeftJoy) {return value(-LJoystick.y)} else {return 0.0}
        }

    fun value(v: Double): Double {
        var command = v

        command = when {
            command > Constants.DEADBAND -> (1 / (1 - Constants.DEADBAND)) * command - (Constants.DEADBAND / (1 - Constants.DEADBAND))
            command < -Constants.DEADBAND -> (1 / (1 - Constants.DEADBAND)) * command + (Constants.DEADBAND / (1 - Constants.DEADBAND))
            else -> 0.0
        }

        var retval = ((1 + 0.01 * Constants.EXPO * (command * command - 1.0)) * command)
        return (retval * (Constants.RATE + (abs(retval) * Constants.RATE * Constants.SUPER_RATE * 0.01)))
    }


}