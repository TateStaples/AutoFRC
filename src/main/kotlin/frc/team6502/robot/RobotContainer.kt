package frc.team6502.robot

import frc.team6502.robot.auto.cv.Vision
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.balls.Shoot
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.auto.Navigator
import kyberlib.input.controller.KXboxController
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    private val gyro = KPigeon(Constants.PIGEON_PORT)  // todo: figure out why PigeonIMU crash mac
    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -2*PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            maxVal = -2.0
            expo = 20.0
            deadband = 0.2
        }

        leftBumper.whileActiveOnce(Shoot())
        rightBumper.whileActiveOnce(Intake())

    }

    val navigation = Navigator(gyro).apply {
        applyMovementRestrictions(Constants.velocity, Constants.acceleration)
        applyKinematics(Drivetrain.kinematics)
    }

    init {
        // initialize subsystems here:
        Drivetrain
        if (Simulation.real) Vision
    }

}