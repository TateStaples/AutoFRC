package frc.team6502.robot

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.general.CommandManager
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.json.Json
import kyberlib.command.KRobot
import kyberlib.math.units.zeroPose


/**
 * Main robot class. Runs the main control loops
 */
class Robot : KRobot() {
    override fun robotInit() {
        // initialize RobotContainer and by extension subsystems
        RobotContainer
    }

    override fun autonomousInit() {
        Constants.AUTO = true
        val drive1 = AutoDrive(Translation2d(0.0, 3.0))  // Drives here
//        val drive2 = AutoDrive(Translation2d(10.0, 3.0))  // drives here
        CommandManager.enqueue(drive1)
    }

    override fun teleopInit() {
        Constants.AUTO = false
        Navigation.pose = zeroPose
    }

    private val slamValues = "slamValues.json"
    private val table = NetworkTableInstance.getDefault().getTable("SLAM")
    private val xEntry = table.getEntry("X")
    private val yEntry = table.getEntry("Y")
    private val thetaEntry = table.getEntry("THETA")
    private val outputTimeEntry = table.getEntry("OUTPUT TIME")
    private var prevOutputTime = -1

    override fun simulationInit() {
    }

    override fun simulationPeriodic() {
        val deserialized = Json.decodeFromString<SlamValues>(slamValues)
        if (prevOutputTime == deserialized.outputImgTime) return
        prevOutputTime = deserialized.outputImgTime
        xEntry.setNumber(deserialized.X)
        yEntry.setNumber(deserialized.Y)
        thetaEntry.setNumber(deserialized.THETA)
        outputTimeEntry.setNumber(deserialized.outputImgTime)
    }
}

data class SlamValues(
    val X:Double, val Y: Double, val THETA: Double, val outputImgTime: Int,
    val inputImgTime: Int
)
