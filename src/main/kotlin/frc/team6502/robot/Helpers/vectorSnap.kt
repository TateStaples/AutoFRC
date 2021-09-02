package frc.team6502.robot.Helpers

import edu.wpi.first.wpilibj.drive.Vector2d
import kotlin.math.abs

fun snap(Main: Vector2d): Vector2d{
    var x = 0.0
    var y = 0.0
    if (abs(Main.x)>0.05) {
        x = Main.x
    }
    if (abs(Main.y)>0.05) {
        x = Main.y
    }
    return Vector2d(x, y)
}