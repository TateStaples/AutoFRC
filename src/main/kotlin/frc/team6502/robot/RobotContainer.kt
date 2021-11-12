package frc.team6502.robot

import edu.wpi.first.wpilibj.RobotBase
import frc.team6502.robot.auto.cv.Photon
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.auto.Navigator
import kyberlib.auto.pathing.Pathfinder
import kyberlib.input.controller.KXboxController
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    private val gyro = KPigeon(Constants.PIGEON_PORT)  // todo: figure out why KPigeon crash mac
    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -2.0
            expo = 20.0
            deadband = 0.2
        }
    }

    val navigation = Navigator(gyro).apply {
        applyMovementRestrictions(Constants.velocity, Constants.acceleration)
        applyKinematics(Drivetrain.kinematics)
    }
    val pathfinder = Pathfinder()

    init {
        // initialize subsystems here:
        Drivetrain
        if (Simulation.real) Photon()
    }

}