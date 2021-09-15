package frc.team6502.robot

import kyberlib.input.controller.KXboxController
import com.ctre.phoenix.sensors.PigeonIMU
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.auto.pathing.PathPlanner
import frc.team6502.robot.subsystems.Drivetrain
import kyberlib.math.units.extensions.degrees
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    /**
     * The location manager of the robot. Tracks location and odometry updates.
     * Also does some pathing.
     */
    val navigation = Navigation(Pose2d(0.0, 0.0, 0.degrees))

    val gyro = PigeonIMU(Constants.PIGEON_PORT)

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
    }

    init {
        // initialize subsystems here:
        Drivetrain
        PathPlanner

        SmartDashboard.putBoolean("AUTO", Constants.AUTO)
        SmartDashboard.putBoolean("DEBUG", Constants.DEBUG)
        SmartDashboard.putBoolean("MECANUM", Constants.MECANUM)
    }

}