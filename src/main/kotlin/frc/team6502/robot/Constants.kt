package frc.team6502.robot

import frc.team6502.kyberlib.math.units.extensions.meters

/**
 * This file holds all important constants throughout the project
 */
object Constants{
    const val DEBUG = false
    const val AUTO = false
    const val MECANUM = false

    // ------ PORTS ------ //

    // drive ports
    const val LEFT_FRONT_ID = 1
    const val LEFT_BACK_ID = 4
    const val RIGHT_FRONT_ID = 3
    const val RIGHT_BACK_ID = 2

    // navigation TODO set up and figure out
    const val PIGEON_PORT = 8

    // shooter ports
    const val INTAKE_ID = 5
    const val SHOOTER_ID = 6

    // ------ CONTROLS ------ //
    // pids TODO tune
    const val DRIVE_P = 0.7
    const val DRIVE_I = 0.1
    const val DRIVE_D = 0.0

    // feed forwards TODO characterize
    const val DRIVE_KS_L = 0.0
    const val DRIVE_KV_L = 0.0
    const val DRIVE_KA_L = 0.0

    // drivetrain setup TODO none of this is right
    const val TRACK_WIDTH = 0.3
    val WHEEL_RADIUS = 0.3.meters
    const val DRIVE_GEAR_RATIO = 1.0 / 10.0

    const val RAMSETE_BETA = 0.7  // TODO I have no idea if these are right
    const val RAMSETE_ZETA = 0.1
}