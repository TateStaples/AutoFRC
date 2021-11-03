package frc.team6502.robot

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.auto.cv.Photon
import frc.team6502.robot.auto.pathing.PathPlanner
import frc.team6502.robot.commands.balls.Intake
import frc.team6502.robot.commands.balls.Shoot
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.input.controller.KXboxController
import kyberlib.sensors.gyros.KPigeon
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    private val gyro = KPigeon(Constants.PIGEON_PORT)

    /**
     * The main user input device of robot
     */
    val controller = KXboxController(0).apply {
        rightX.apply {
            rate = -5 * PI
            expo = 73.0
            deadband = 0.1
        }

        // throttle
        leftY.apply {
            rate = -12.0
            expo = 20.0
            deadband = 0.2
        }

        rightBumper.whileActiveOnce(Shoot)
        leftBumper.whileActiveOnce(Intake)
    }

    init {
        // initialize subsystems here:
        Navigation.apply { gyro = RobotContainer.gyro }
        Drivetrain
        PathPlanner
        Photon

        SmartDashboard.putBoolean("AUTO", Constants.AUTO)
        SmartDashboard.putBoolean("DEBUG", Constants.DEBUG)
        SmartDashboard.putBoolean("MECANUM", Constants.MECANUM)
    }

}