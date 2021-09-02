package frc.team6502.robot

import java.time.*
import kotlin.concurrent.timer
import kotlin.concurrent.timerTask
class SnapControls(Toggled: Boolean, SnapRange: Double) {
    var IsOn: Boolean = false
    var SnapDist: Double = 0.0
    init {
        IsOn = Toggled
        SnapDist = SnapRange
    }
}
object APrefrences {
    //    ---- Controls ----
    /// Big Stuff
    var GeneralControls = true
    var Forward = true      // Robot can move forward
    var Backward = true     // Robot can move backward
    var Strafe = true       // Robot can strafe (side to side)
    var Turning = true      // Robot can turn

    /// QoL
    var CenterSnap: SnapControls = SnapControls(false, 0.05) //(_Boolean_, ) weather to snap to center; (, _Double_) how far to snap to (0,0) from
    var cardinalSnap: SnapControls = SnapControls(false, 0.1) // not implemented
    var dualCardinalSnap: SnapControls = SnapControls(false, 0.025)


    //    ---- Controller Options ----
    val LeftJoy = true
    val RightJoy = true

    val Rumble = true

    //    ---- Sensors ----
    val BuiltinAccelerometer = false

    //    ---- Limiters ----
    /// Speed Limits
    val GeneralSpeed: Double = 1.0 // not Implemented
    val shooterMultiplier: Double = 1.0
    val intakeMultiplier: Double = 0.1
    val ReverseIntakeSpeed: Double = 0.1


    //    ---- Debugging ----
    ///Generally keep these off
    val ControllerPositions = false
    val DebugMotors = false


    fun Setup() {
        if (!GeneralControls) {
            Forward = GeneralControls
            Backward = GeneralControls
            Strafe = GeneralControls
            Turning = GeneralControls
        }
    }
}