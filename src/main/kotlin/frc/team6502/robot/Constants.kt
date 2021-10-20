package frc.team6502.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kyberlib.math.units.extensions.inches
import kyberlib.math.units.extensions.meters
import kyberlib.math.units.extensions.metersPerSecond

/**
 * This file holds all important constants throughout the project
 */
object Constants{
    const val DEBUG = true
    var AUTO = false
        set(value) {
            field = value
            SmartDashboard.putBoolean("AUTO", value)
        }
    const val MECANUM = true
    const val VISION = true

    // ------ PORTS ------ //

    // drive ports
    const val LEFT_FRONT_ID = 2
    const val LEFT_BACK_ID = 3
    const val RIGHT_FRONT_ID = 4
    const val RIGHT_BACK_ID = 1

    // navigation
    const val PIGEON_PORT = 6

    // shooter ports
    const val INTAKE_ID = 5
    const val SHOOTER_ID = 6

    // ------ CONTROLS ------ //
    // pids TODO tune
    const val DRIVE_P = 0.7
    const val DRIVE_I = 0.1
    const val DRIVE_D = 0.0

    // feed forwards TODO characterize
    const val DRIVE_KS = 0.0
    const val DRIVE_KV = 0.0
    const val DRIVE_KA = 0.0

    // drivetrain setup TODO: track width
    const val TRACK_WIDTH = 0.3
    val WHEEL_RADIUS = 2.inches
    const val DRIVE_GEAR_RATIO = 1.0 / 20.0

    const val RAMSETE_BETA = 0.7  // TODO I have no idea if these are right
    const val RAMSETE_ZETA = 0.1

    // TODO: mess with this - these are max values
    val acceleration = 1.metersPerSecond
    val velocity = 1.metersPerSecond
}