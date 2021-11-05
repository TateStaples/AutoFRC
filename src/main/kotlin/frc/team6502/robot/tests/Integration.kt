package frc.team6502.robot.tests

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.team6502.robot.Constants
import frc.team6502.robot.subsystems.Drive
import kyberlib.auto.Navigator
import kyberlib.command.KRobot
import kyberlib.input.controller.KXboxController
import kyberlib.math.units.extensions.metersPerSecond
import kyberlib.sensors.gyros.KPigeon
import kyberlib.simulation.Simulation
import kyberlib.simulation.field.KField2d
import kotlin.math.PI

class Integration : KRobot() {
    init {
        Drive
        if (!Simulation.real) {
            Simulation.instance.include(Drive)
            Drive.setupSim()
        }
    }
    val gyro = KPigeon(Constants.PIGEON_PORT)
    val controller = KXboxController(0).apply {
        rightX.apply {
            maxVal = -3 * PI
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

    val navigation = Navigator(gyro)

    override fun teleopPeriodic() {
        val forward = controller.leftY.value
        val turn = controller.rightX.value
        SmartDashboard.putNumber("forward", forward)
        SmartDashboard.putNumber("turn", turn)
        Drive.chassisSpeeds = ChassisSpeeds(forward, 0.0, turn)
    }

    override fun robotPeriodic() {
        KField2d.robotPose = navigation.pose
    }

    override fun simulationPeriodic() {
        val forward = controller.leftY.value.coerceAtMost(Constants.velocity.metersPerSecond)
        val turn = controller.rightX.value
        SmartDashboard.putNumber("forward", forward)
        SmartDashboard.putNumber("turn", turn)
        Drive.chassisSpeeds = ChassisSpeeds(forward, 0.0, turn)
    }

}