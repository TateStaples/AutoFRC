package Utilities.input.controller

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.team6502.kyberlib.input.KAxis
import frc.team6502.kyberlib.input.KController
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team6502.kyberlib.input.AxisButton
import java.util.function.BooleanSupplier

class KXboxController(port: Int) : KController(port) {
    val triggerSensitivity = 0.2

    val leftX = KAxis { joystick.getRawAxis(0) }
    val leftY = KAxis { joystick.getRawAxis(1) }

    val rightX = KAxis{ joystick.getRawAxis(4) }
    val rightY = KAxis{ joystick.getRawAxis(5) }

    val aButton = JoystickButton(joystick, 1)
    val bButton = JoystickButton(joystick, 2)
    val xButton = JoystickButton(joystick, 3)
    val yButton = JoystickButton(joystick, 4)

    val leftBumper = JoystickButton(joystick, 5)  // these might be the menu buttons
    val rightBumper = JoystickButton(joystick, 6)

    val leftTrigger = AxisButton(joystick, 2) {value: Double -> value > triggerSensitivity}
    val rightTrigger = AxisButton(joystick, 3) {value: Double -> value > triggerSensitivity}

    private val DPad
        get() = joystick.getPOV()  // up = 0, 45º increments clockwise, none = -1

    val rightDPad = Trigger(BooleanSupplier({ DPad in 1..179 }))  // 90
    val leftDPad = Trigger(BooleanSupplier { DPad > 180 })  // 270
    val upDPad = Trigger(BooleanSupplier { DPad != 0 && (DPad < 90 || DPad > 270) })  // 0
    val downDPad = Trigger(BooleanSupplier { DPad in 91..269 })  // 180

    var rumbleLeft = 0.0
        set(value) {
            joystick.setRumble(GenericHID.RumbleType.kLeftRumble, value)
            field = value
        }
    var rumbleRight = 0.0
        set(value) {
            joystick.setRumble(GenericHID.RumbleType.kRightRumble, value)
            field = value
        }
    var rumble: Double
        get() = rumbleLeft.coerceAtLeast(rumbleRight)
        set(value) {
            rumbleLeft = value
            rumbleRight = value
        }
}
