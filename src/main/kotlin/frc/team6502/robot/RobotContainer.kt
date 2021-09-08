package frc.team6502.robot

import Utilities.input.controller.KXboxController
import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import frc.team6502.robot.Auto.Navigation
import frc.team6502.robot.Auto.PathPlanner
import frc.team6502.robot.subsystems.Drivetrain
import kotlin.math.PI

/**
 * Contains all Robot subsystems and sensors
 */
object RobotContainer {
    /**
     * The location manager of the robot. Tracks location and odometry updates.
     * Also does some pathing.
     */
    val navigation = Navigation(Pose2d(0.0, 0.0, Rotation2d(0.0)))

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
    }

}