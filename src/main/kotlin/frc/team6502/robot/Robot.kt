package frc.team6502.robot

import edu.wpi.first.cameraserver.CameraServer
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.geometry.Translation2d
import frc.team6502.robot.auto.Navigation
import frc.team6502.robot.commands.drive.AutoDrive
import frc.team6502.robot.commands.general.CommandManager
import kotlinx.serialization.Serializable
import kotlinx.serialization.decodeFromString
import kotlinx.serialization.encodeToString
import kotlinx.serialization.json.Json
import kyberlib.command.KRobot
import kyberlib.math.units.zeroPose
import kyberlib.runCommand
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.imgcodecs.Imgcodecs
import java.io.File


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
    private var prevOutputTime = -1.0

    override fun simulationInit() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
        val saveFile = File(slamValues)
        if (!saveFile.exists()) {
            val defaultVals = SlamValues(0.0, 0.0, 0.0,0.0, 0.0, true)
            val jsonString = Json.encodeToString(defaultVals)
            File(slamValues).writeText(jsonString)
        }
//        "frcMonocular.exe".runCommand(File("./UcoSlam/"))
    }

    override fun simulationPeriodic() {
        // read to SLAM output
        val deserialized = Json.decodeFromString<SlamValues>(slamValues)
        if (prevOutputTime == deserialized.outputImgTime) return
        prevOutputTime = deserialized.outputImgTime
        xEntry.setNumber(deserialized.X)
        yEntry.setNumber(deserialized.Y)
        thetaEntry.setNumber(deserialized.THETA)
        outputTimeEntry.setNumber(deserialized.outputImgTime)

        // write the SLAM input
        val mat = Mat()
        CameraServer.getInstance().video.grabFrame(mat)
        Imgcodecs.imwrite("slam.jpg", mat)
        deserialized.newImgTime = Timer.getFPGATimestamp()
        val jsonString = Json.encodeToString(deserialized)
        println(jsonString)
        File(slamValues).writeText(jsonString)
    }
}

@Serializable
data class SlamValues(
    var X:Double, var Y: Double, var THETA: Double, var outputImgTime: Double,
    var newImgTime: Double, var running: Boolean
)
